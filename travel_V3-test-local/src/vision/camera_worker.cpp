#include "project_paths.hpp"
#include <QDir>
#include <QTcpSocket>
#include <QUrl>
#include <QtEndian>
#include "camera_worker.hpp"
#include "frame_context.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/cuda.hpp>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <algorithm>

namespace CameraAssist {
namespace {
int checkedIoctl(int fd,unsigned long request,void* arg) {
    int result;do{result=ioctl(fd,request,arg);}while(result<0&&errno==EINTR);return result;
}
class VideoDevice {
    struct Buffer { void* data=MAP_FAILED; std::size_t size=0; };
    int fd_=-1;bool streaming_=false;std::vector<Buffer> buffers_;
    unsigned format_=0;int stride_=0;
public:
    int width=0,height=0;
    explicit VideoDevice(const CameraSettings& settings) {
        try {
            fd_=open(settings.device.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
            if(fd_<0)throw std::runtime_error("打开摄像头失败："+std::string(strerror(errno)));
            v4l2_capability capability{};
            if(checkedIoctl(fd_,VIDIOC_QUERYCAP,&capability)<0)throw std::runtime_error("设备不是 V4L2 摄像头。");
            const auto flags=(capability.capabilities&V4L2_CAP_DEVICE_CAPS)?capability.device_caps:capability.capabilities;
            if(!(flags&V4L2_CAP_VIDEO_CAPTURE)||!(flags&V4L2_CAP_STREAMING))throw std::runtime_error("设备不支持视频采集；请选择 video-index0。");
            v4l2_format fmt{};fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmt.fmt.pix.width=settings.width;fmt.fmt.pix.height=settings.height;
            fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_MJPEG;fmt.fmt.pix.field=V4L2_FIELD_ANY;
            if(checkedIoctl(fd_,VIDIOC_S_FMT,&fmt)<0)throw std::runtime_error("无法设置摄像头格式。");
            width=fmt.fmt.pix.width;height=fmt.fmt.pix.height;format_=fmt.fmt.pix.pixelformat;stride_=fmt.fmt.pix.bytesperline;
            if(width<=0||height<=0||(format_!=V4L2_PIX_FMT_MJPEG&&format_!=V4L2_PIX_FMT_YUYV))throw std::runtime_error("摄像头需支持 MJPEG 或 YUYV。");
            v4l2_streamparm params{};params.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            params.parm.capture.timeperframe.numerator=1;params.parm.capture.timeperframe.denominator=30;
            checkedIoctl(fd_,VIDIOC_S_PARM,&params);
            v4l2_requestbuffers request{};request.count=4;request.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;request.memory=V4L2_MEMORY_MMAP;
            if(checkedIoctl(fd_,VIDIOC_REQBUFS,&request)<0||request.count<2)throw std::runtime_error("无法申请摄像头缓冲区。");
            buffers_.resize(request.count);
            for(unsigned i=0;i<request.count;++i) {
                v4l2_buffer buf{};buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;buf.memory=V4L2_MEMORY_MMAP;buf.index=i;
                if(checkedIoctl(fd_,VIDIOC_QUERYBUF,&buf)<0)throw std::runtime_error("无法查询摄像头缓冲区。");
                buffers_[i]={mmap(nullptr,buf.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,buf.m.offset),buf.length};
                if(buffers_[i].data==MAP_FAILED||checkedIoctl(fd_,VIDIOC_QBUF,&buf)<0)throw std::runtime_error("无法映射摄像头缓冲区。");
            }
            int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if(checkedIoctl(fd_,VIDIOC_STREAMON,&type)<0)throw std::runtime_error("摄像头启动失败，可能正在被其他程序使用。");
            streaming_=true;
        }catch(...){close();throw;}
    }
    void close() {
        if(streaming_){int type=V4L2_BUF_TYPE_VIDEO_CAPTURE;checkedIoctl(fd_,VIDIOC_STREAMOFF,&type);streaming_=false;}
        for(auto& b:buffers_)if(b.data!=MAP_FAILED){munmap(b.data,b.size);b.data=MAP_FAILED;}
        if(fd_>=0){::close(fd_);fd_=-1;}
    }
    ~VideoDevice(){close();}
    bool read(cv::Mat& image,double& captured,std::string& quality,QByteArray& originalJpeg) {
        fd_set set;FD_ZERO(&set);FD_SET(fd_,&set);timeval timeout{0,100000};
        const int ready=select(fd_+1,&set,nullptr,nullptr,&timeout);
        if(ready<0){if(errno==EINTR)return false;throw std::runtime_error("摄像头等待失败。");}
        if(!ready)return false;
        std::vector<uchar> bytes;
        // Drain only the buffers already queued. Never build an inference queue.
        for(std::size_t i=0;i<buffers_.size();++i) {
            v4l2_buffer buf{};buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;buf.memory=V4L2_MEMORY_MMAP;
            if(checkedIoctl(fd_,VIDIOC_DQBUF,&buf)<0){if(errno==EAGAIN)break;throw std::runtime_error("摄像头断开或读取失败。");}
            if(buf.index>=buffers_.size()||buf.bytesused>buffers_[buf.index].size)throw std::runtime_error("摄像头缓冲区长度异常。");
            if(buf.flags&V4L2_BUF_FLAG_ERROR) {
                if(checkedIoctl(fd_,VIDIOC_QBUF,&buf)<0)throw std::runtime_error("摄像头缓冲区归还失败。");
                continue;
            }
            const auto* ptr=static_cast<const uchar*>(buffers_[buf.index].data);
            bytes.assign(ptr,ptr+buf.bytesused);
            const double received=RadarTime::monotonicSeconds();
            const double stamp=buf.timestamp.tv_sec+buf.timestamp.tv_usec/1e6;
            if((buf.flags&V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)&&stamp>0&&received-stamp>=-.02&&received-stamp<2) {
                captured=stamp;quality="v4l2_monotonic";
            } else {captured=received;quality="receive_estimate";}
            if(checkedIoctl(fd_,VIDIOC_QBUF,&buf)<0)throw std::runtime_error("摄像头缓冲区归还失败。");
        }
        if(bytes.empty())return false;
        if(format_==V4L2_PIX_FMT_MJPEG)image=cv::imdecode(bytes,cv::IMREAD_COLOR);
        else {
            const int stride=std::max(stride_,width*2);
            if(bytes.size()<static_cast<std::size_t>(stride)*height)throw std::runtime_error("YUYV 图像长度不足。");
            cv::cvtColor(cv::Mat(height,width,CV_8UC2,bytes.data(),stride),image,cv::COLOR_YUV2BGR_YUYV);
        }
        if(format_==V4L2_PIX_FMT_MJPEG)originalJpeg=QByteArray(reinterpret_cast<const char*>(bytes.data()),int(bytes.size()));
        if(image.empty())return false;
        return true;
    }
};

// Pull one fresh JPEG at a time. The Windows helper keeps capturing while DNN
// runs, so a slow detector never creates a queue of old video frames.
class BridgeDevice {
    QTcpSocket socket_;
    const std::atomic<bool>& running_;
    QByteArray pending_;
    quint32 length_=0;
    bool requested_=false;
public:
    int width,height;
    BridgeDevice(const CameraSettings& settings,const std::atomic<bool>& running)
        :running_(running),width(settings.width),height(settings.height) {
        const QUrl url(QString::fromStdString(settings.device));
        if(!url.isValid()||url.scheme()!="tcp"||url.host().isEmpty()||url.port()<=0||
           url.port()>65535||!url.userInfo().isEmpty()||!url.path().isEmpty()||url.hasQuery()||url.hasFragment())
            throw std::runtime_error("本机摄像头地址无效，应为 tcp://主机:端口。");
        socket_.connectToHost(url.host(),quint16(url.port()));
        const double deadline=RadarTime::monotonicSeconds()+3;
        while(running_&&socket_.state()!=QAbstractSocket::ConnectedState) {
            if(socket_.waitForConnected(100))break;
            if(socket_.state()==QAbstractSocket::UnconnectedState||RadarTime::monotonicSeconds()>deadline)
                throw std::runtime_error("无法连接本机摄像头桥接："+socket_.errorString().toStdString());
        }
    }
    bool read(cv::Mat& image,double& captured,std::string& quality,QByteArray& originalJpeg) {
        if(!running_)return false;
        if(socket_.state()!=QAbstractSocket::ConnectedState)
            throw std::runtime_error("本机摄像头桥接已断开。");
        if(!requested_) {
            if(socket_.write("F",1)!=1)throw std::runtime_error("本机摄像头请求发送失败。");
            socket_.flush();requested_=true;
        }
        if(!socket_.bytesAvailable()&&!socket_.waitForReadyRead(100)) {
            if(socket_.state()!=QAbstractSocket::ConnectedState)
                throw std::runtime_error("本机摄像头桥接已断开。");
            return false;
        }
        // Preserve partial headers/payloads across the interruptible reads.
        if(!length_) {
            pending_+=socket_.read(4-pending_.size());
            if(pending_.size()!=4)return false;
            length_=qFromBigEndian<quint32>(reinterpret_cast<const uchar*>(pending_.constData()));
            pending_.clear();
            if(length_>8*1024*1024)throw std::runtime_error("本机摄像头图像长度异常。");
            if(!length_){requested_=false;return false;} // No newer frame yet.
        }
        pending_+=socket_.read(length_-pending_.size());
        if(pending_.size()!=int(length_))return false;
        captured=RadarTime::monotonicSeconds();quality="receive_estimate";
        const cv::Mat encoded(1,pending_.size(),CV_8UC1,pending_.data());
        image=cv::imdecode(encoded,cv::IMREAD_COLOR);
        if(image.empty())throw std::runtime_error("本机摄像头 JPEG 解码失败。");
        originalJpeg=std::move(pending_);pending_.clear();length_=0;requested_=false;
        width=image.cols;height=image.rows;
        return true;
    }
};
}

std::vector<Person> detectPeople(cv::dnn::Net& net,const cv::Mat& bgr) {
    constexpr int size=640;
    const double scale=std::min(double(size)/bgr.cols,double(size)/bgr.rows);
    cv::Mat resized;cv::resize(bgr,resized,cv::Size(cvRound(bgr.cols*scale),cvRound(bgr.rows*scale)));
    const int left=(size-resized.cols)/2,top=(size-resized.rows)/2;
    cv::Mat padded(size,size,CV_8UC3,cv::Scalar(114,114,114));resized.copyTo(padded(cv::Rect(left,top,resized.cols,resized.rows)));
    net.setInput(cv::dnn::blobFromImage(padded,1.0/255,cv::Size(size,size),cv::Scalar(),true,false));
    cv::Mat output=net.forward();
    if(output.dims!=3||output.size[0]!=1||output.size[1]!=84||output.type()!=CV_32F)
        throw std::runtime_error("模型输出不兼容：请选择 COCO YOLO11 检测 ONNX（1×84×N，640 输入）。");
    const int count=output.size[2];const float* p=output.ptr<float>();
    std::vector<cv::Rect> boxes;std::vector<float> scores;
    for(int i=0;i<count;++i) {
        const float score=p[4*count+i];if(!std::isfinite(score)||score<.4f)continue;
        bool person=true;for(int c=5;c<84;++c)if(p[c*count+i]>score){person=false;break;}if(!person)continue;
        const double cx=p[i],cy=p[count+i],w=p[2*count+i],h=p[3*count+i];
        if(!std::isfinite(cx)||!std::isfinite(cy)||!std::isfinite(w)||!std::isfinite(h)||w<=0||h<=0)continue;
        cv::Rect box(cvRound((cx-w/2-left)/scale),cvRound((cy-h/2-top)/scale),cvRound(w/scale),cvRound(h/scale));
        box&=cv::Rect(0,0,bgr.cols,bgr.rows);if(box.width<8||box.height<16)continue;
        boxes.push_back(box);scores.push_back(score);
    }
    std::vector<int> keep;cv::dnn::NMSBoxes(boxes,scores,.4f,.45f,keep);
    std::vector<Person> result;for(int i:keep)result.push_back({cv::Rect2f(boxes[i]),scores[i],0,false});
    return result;
}
CameraWorker::~CameraWorker(){stop();}
void CameraWorker::start(const CameraSettings& settings) {
    stop();{std::lock_guard<std::mutex> lock(mutex_);latest_.reset();}
    running_=true;
    try{worker_=std::thread(&CameraWorker::run,this,settings);}catch(...){running_=false;throw;}
}
void CameraWorker::stop(){running_=false;if(worker_.joinable())worker_.join();std::lock_guard<std::mutex> lock(mutex_);latest_.reset();}
std::shared_ptr<const CameraPacket> CameraWorker::latest()const{std::lock_guard<std::mutex>lock(mutex_);return latest_;}
void CameraWorker::publish(std::shared_ptr<CameraPacket> packet){std::lock_guard<std::mutex>lock(mutex_);latest_=std::move(packet);}
void CameraWorker::run(CameraSettings settings) {
    try {
        cv::setNumThreads(2);
        auto net=cv::dnn::readNetFromONNX(settings.model);
        if(net.empty())throw std::runtime_error("无法加载人物检测模型。");
        bool cuda=settings.cuda&&cv::cuda::getCudaEnabledDeviceCount()>0;
        if(cuda){
            // This Jetson's OpenCV 4.10 CUDA graph fusion asserts on YOLO11.
            // Unfused CUDA inference works; keep CPU fusion unchanged on fallback.
            net.enableFusion(false);
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
        }
        std::unique_ptr<VideoDevice> video;
        std::unique_ptr<BridgeDevice> bridge;
        if(settings.device.rfind("tcp://",0)==0)bridge=std::make_unique<BridgeDevice>(settings,running_);
        else video=std::make_unique<VideoDevice>(settings);
        // Prime bridge input so session metadata uses the actual camera size.
        cv::Mat firstImage;double firstCaptured=0;std::string firstQuality;QByteArray firstJpeg;
        if(bridge) {
            const double deadline=RadarTime::monotonicSeconds()+3;
            while(running_&&!bridge->read(firstImage,firstCaptured,firstQuality,firstJpeg))
                if(RadarTime::monotonicSeconds()>deadline)throw std::runtime_error("摄像头超过 3 秒没有图像。");
        }
        PersonTracker tracker;
        if(settings.record){
            recorder_.start(QString::fromStdString(RadarPaths::resolve().cameraRecordings().u8string()));
            recorder_.evidence(QJsonObject{{"kind","session_start"},{"device",QString::fromStdString(settings.device)},
                {"model",QString::fromStdString(settings.model)},{"width",bridge?bridge->width:video->width},{"height",bridge?bridge->height:video->height},{"version","V1024-2158"}});
        }
        auto session=static_cast<std::uint64_t>(RadarTime::wallSeconds()*1e6);
        std::uint64_t sequence=0;double last_read=RadarTime::monotonicSeconds();
        while(running_) {
            cv::Mat bgr;double captured=0;std::string quality;QByteArray originalJpeg;
            bool received=false;
            if(!firstImage.empty()) {
                bgr=std::move(firstImage);captured=firstCaptured;quality=std::move(firstQuality);originalJpeg=std::move(firstJpeg);received=true;
            } else received=bridge?bridge->read(bgr,captured,quality,originalJpeg):video->read(bgr,captured,quality,originalJpeg);
            if(!received) {
                if(RadarTime::monotonicSeconds()-last_read>3)throw std::runtime_error("摄像头超过 3 秒没有图像。");
                continue;
            }
            last_read=RadarTime::monotonicSeconds();
            auto packet=std::make_shared<CameraPacket>();auto& f=packet->frame;
            f.session=session;f.sequence=++sequence;f.device=settings.device;f.width=bgr.cols;f.height=bgr.rows;
            f.captured_monotonic=captured;f.captured_wall=RadarTime::wallSeconds()-(RadarTime::monotonicSeconds()-captured);f.time_quality=quality;
            const auto begin=RadarTime::monotonicSeconds();std::vector<Person> detections;
            try{detections=detectPeople(net,bgr);}catch(const cv::Exception&) {
                if(!cuda)throw;
                cuda=false;net=cv::dnn::readNetFromONNX(settings.model);
                net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                tracker.reset();f.session=++session;sequence=0;f.sequence=++sequence;
                detections=detectPeople(net,bgr);
            }
            f.completed_monotonic=RadarTime::monotonicSeconds();packet->inference_ms=(f.completed_monotonic-begin)*1000;
            f.people=tracker.update(detections,captured);packet->status=cuda?"YOLO11 / CUDA FP16":"YOLO11 / CPU";
            cv::Mat rgb;cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
            packet->image=QImage(rgb.data,rgb.cols,rgb.rows,static_cast<int>(rgb.step),QImage::Format_RGB888).copy();
            if(settings.record)recorder_.frame(f,packet->image,std::move(originalJpeg));
            if(running_)publish(std::move(packet));
        }
    }catch(const std::exception& e){auto error=std::make_shared<CameraPacket>();error->error=e.what();publish(error);}
    recorder_.stop();
    running_=false;
}
}
