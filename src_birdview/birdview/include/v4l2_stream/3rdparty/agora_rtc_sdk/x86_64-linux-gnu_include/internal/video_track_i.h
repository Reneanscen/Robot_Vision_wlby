//
//  Agora Media SDK
//
//  Created by Rao Qi in 2019.
//  Copyright (c) 2019 Agora IO. All rights reserved.
//
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "AgoraBase.h"
#include "AgoraMediaBase.h"
#include "AgoraOptional.h"
#include "NGIAgoraVideoTrack.h"
#include "api/transport/network_types.h"
#include "api/video/video_content_type.h"
#include "call/rtp_config.h"

#include "rtc_connection_i.h"
#include "track_stat_i.h"
#include "video_config_i.h"
#include "common_defines.h"
#include "video_node_i.h"
#include "utils/thread/base_worker.h"
#include "utils/thread/thread_control_block.h"
#include "main/core/video/multi_stream_subscribe_interface.h"
#include "facilities/media_config/policy_chain/media_config_policy_chain.h"
#include "facilities/tools/weak_observers.h"
#include "facilities/media_config/policy_chain/general_val_policy_chain.h"
#include "main/core/video/strategy_framework/module_controller/video_module_control_aspect.h"
#include <utils/object/object_table.h>
#include "main/core/video/stats_and_events/video_stats_events_base.h"
namespace webrtc {
  class IFecMethodFactoryInterface;
  class ISmoothRender;
  class IRsfecCodecFactoryInterface;
  class IAutoAdjustHarq;
}

namespace agora {
namespace rtc {

class VideoNodeRtpSink;
class VideoNodeRtpSource;
class VideoTrackConfigurator;
class ProactiveCaller;
class IModuleControlPanel;

enum class InternalVideoSourceType : unsigned {
  None = 0,
  Camera = 1,
  Custom = 2,
  Screen = 3,
  CustomYuvSource = 4,
  CustomEncodedImageSource = 5,
  CustomPacketSource = 6,
  MixedSource = 7,
  TranscodedSource = 8,
};

enum VideoModuleId {
  VideoModuleCapture = 1,
  VideoModulePreprocess,
  VideoModuleEncode,
  VideoModuleNetwork,
  VideoModuleDecode,
  VideoModulePostprocess,
  VideoModuleRender,
  VideoModulePipeline,
  VideoModuleQoE,
};

enum VideoAvailabilityLevel {
  VideoAvailabilityLevel1 = 1,  // Completely unusable.
  VideoAvailabilityLevel2,      // Usable but with very poor experience.
  VideoAvailabilityLevel3,      // Usable but with poor experience.
};

// Events report. New enum can be added but do not change the existing value!
enum VideoPipelineEvent {
  kVideoUplinkEventStaticFrames = 1,  // Continous static frames, maybe green/black picktures.
};

enum VideoQoeEvent {
  kVideoQoeCriticalDrop = 1,
  kVideoQoe600msFreezeDrop = 2,
  kVideoQoe200msFreezeDrop = 3,
  kVideoQoeFpsSubstandard = 4,
  kVideoTimestampException = 5,
};

// Events report. New enum can be added but do not change the existing value!
enum VideoProcessEvent {
  kVideoProcessEventNone = 0,

  // These events report will be throttled, refer to VideoEngine::doReportVideoEvent().
  kVideoProcessEventPreprocessEnqueueFailure = 1000,
  kVideoProcessEventPreprocessFrameFailure = 1001,
  kVideoProcessEventPreprocessNoIncomingFrame = 1002,  // No incoming frame for builtin VPM module
  kVideoProcessEventPreprocessCongested = 1003,
};

// report hardware codec availability event.
enum VideoCodecAvailableEvent {
  kVideoHwH265EncoderAvailable = 2000,
  kVideoHwH264EncoderHighProfileAvailable = 2001,
};

enum VideoDumpMode {
  VIDEO_DUMP_DEFAULT = 0, // Dump all.
  VIDEO_DUMP_CAPTURED_YUV = (1 << 0), // Dump YUV after video capturing.
  VIDEO_DUMP_FILTERED_YUV = (1 << 1), // Dump YUV before video encoding.
  VIDEO_DUMP_ENCODED_STREAM = (1 << 2), // Dump stream after video encoding.
  VIDEO_DUMP_RECEIVED_STREAM = (1 << 3), // Dump stream before video decoding.
  VIDEO_DUMP_DECODED_YUV = (1 << 4), // Dump YUV after video decoding.
  VIDEO_DUMP_RENDERED_YUV = (1 << 5), // Dump YUV before video rendering.
};

struct VideoAvailabilityIndicator {
  VideoAvailabilityLevel level;
  VideoModuleId module;
  int code;
  uid_t uid;
  int extra;
  std::vector<agora::rtc::QoEDropInfo> extra2;
  std::vector<agora::rtc::VideoTimestampExceptionInfo> ts_exception_info;
};

struct VideoQoEAnalyzerParameter {
  bool qoe_analyzer_enable = false;
  int qoe_critical_report_max_times = 0;
  int qoe_high_report_max_times = 0;
  int qoe_normal_report_max_times = 0;
  int qoe_report_strategy = 0;
  int qoe_timing_strategy_report_period = 0;
};

class IVideoTrackObserver : public std::enable_shared_from_this<IVideoTrackObserver> {
 public:
  virtual ~IVideoTrackObserver() = default;
  virtual void onLocalVideoStateChanged(int id,
                                        LOCAL_VIDEO_STREAM_STATE state,
                                        LOCAL_VIDEO_STREAM_ERROR errorCode,
                                        int timestamp_ms) {}

  virtual void onRemoteVideoStateChanged(uid_t uid,
                                         REMOTE_VIDEO_STATE state,
                                         REMOTE_VIDEO_STATE_REASON reason,
                                         int timestamp_ms) {}

  virtual void onFirstVideoFrameRendered(int id, uid_t uid, int width, int height, int timestamp_ms) {}

  virtual void onFirstVideoFrameDecoded(std::string cid, uid_t uid, uint32_t ssrc, int width, int height, int timestamp_ms) {}

  virtual void onFirstVideoKeyFrameReceived(uid_t uid, uint64_t timestamp, const webrtc::FirstVideoFrameStreamInfo &streamInfo) {}

  virtual void onVideoContentChanged(uid_t uid, agora::VideoContentType newType, agora::VideoContentSubType newSubtype) {};

  virtual void onSourceVideoSizeChanged(uid_t uid,
                                        int width, int height,
                                        int rotation, int timestamp_ms) {}
  virtual void onSendSideDelay(int id, int send_delay) {}
  virtual void onRecvSideDelay(uid_t uid, int recv_delay) {}
  virtual void onRecvSideFps(uid_t uid, int fps) {}
  virtual void onEncoderConfigurationChanged(const std::vector<VideoConfigurationEx>& config) {}
  virtual void onCameraFacingChanged(int facing) {}
  virtual void onViewSizeChanged(uid_t uid, agora::utils::object_handle view, int width, int height) {}
  virtual void OnSetRexferParams(bool fec_rexfer, float rexfer_alpha) {}
  virtual void OnRexferStatusUpdated(bool status, int32_t target_bitrate) {}
  virtual void OnRequestCodecFallback() {}
  virtual void OnNotifyDepartedFrame(uid_t uid, int picture_id) {}
  virtual void onCameraInfoListChanged(CameraInfoList cameraInfoList) {}
  virtual void OnRoiStatusUpdated(bool status){}
  virtual void OnEncoderStatusUpdate(webrtc::VideoCodecType codec_type,
                                     webrtc::HW_ENCODER_ACCELERATING_STATUS hw_accelerate_status) {};
  virtual void onVideoAvailabilityIndicatorEvent(VideoAvailabilityIndicator indicator) {}
  virtual void onVideoSizeChanged(int id, uid_t uid, int width, int height, int rotation) {}

  virtual void onLocalAddVideoFilter(int track_id, std::string filter_name, bool enabled){}
  virtual void onLocalFilterStatusChanged(int track_id, std::string filter_name, bool enabled){}
  virtual void onRemoteAddVideoFilter(std::string cid, uid_t uid, uint32_t ssrc, std::string filter_name, bool enabled){}
  virtual void onRemoteFilterStatusChanged(std::string cid, uid_t uid, uint32_t ssrc, std::string filter_name, bool enabled, bool isDisableMe = false){}
};

struct LocalVideoTrackStatsEx {
  LocalVideoTrackStats local_video_stats;
  int sent_loss_ratio;
};

class ILocalVideoTrackEx : public ILocalVideoTrack,
                           public VideoLocalTrackControlAspect {
 public:
  enum DetachReason { MANUAL, TRACK_DESTROY, NETWORK_DESTROY };

  // keep the same as webrtc::RsfecConfig
  struct RsfecConfig {
    std::vector<int> fec_protection_factor;
    std::vector<std::vector<int>> fec_ratioLevel;
    std::vector<int> fec_rttThreshold;
    bool pec_enabled;
  };

  struct AttachInfo {
    uint32_t uid;
    uint32_t cid;
    VideoNodeRtpSink* network;
    WeakPipelineBuilder builder;
    uint64_t stats_space;
    CongestionControlType cc_type;
    bool enable_two_bytes_extension;
    webrtc::RsfecConfig rsfec_config;

    // hardware encoder related
    std::string enable_hw_encoder;
    std::string hw_encoder_provider;
    Optional<bool> low_stream_enable_hw_encoder;
    Optional<int> minscore_for_swh265enc;

    // video config
    VideoNodeEncoderEx::OPSParametersCollection ops_parameters;
    std::shared_ptr<webrtc::IFecMethodFactoryInterface> fec_method_factory;
    std::shared_ptr<webrtc::IAutoAdjustHarq> auto_adjust_harq;
    int harq_version;
    int32_t fec_outside_bandwidth_ratio;
    bool enable_minor_stream_vqc = false;
    bool enable_minor_stream_fec = false;
    bool enable_minor_stream_fec_outside_ratio = false;
    bool enable_minor_stream_intra_request = false;
    
    int fec_method;
    int dm_wsize;
    int dm_maxgc;
    std::string switch_to_rq;
    bool dm_lowred;

    int32_t minimum_fec_level;
    int fec_fix_rate;
    int largest_ref_distance;
    bool enable_check_for_disable_fec;
    bool enable_quick_intra_high_fec = false; 
    absl::optional<int> max_inflight_frame_count_pre_processsing;

    // for intra request
    Optional<uint32_t> av_enc_intra_key_interval;

    Optional<uint32_t> av_enc_bitrate_adjustment_type;

    // enable video diagnose
    bool enable_video_send_diagnose;
    int max_packet_size;
    // video codec alignment
    Optional<uint32_t> hw_encoder_width_alignment;
    Optional<uint32_t> hw_encoder_height_alignment;
    Optional<bool> hw_encoder_force_alignment;
    // video decode capablitys
    uint8_t negotiated_video_decode_caps;
    // hw video encode configure
    std::string hw_encoder_fotmat_config;
    Optional<uint32_t> hw_enc_hevc_exceptions;

    int hw_capture_delay;
    uint32_t sync_peer_uid;

    Optional<SIMULCAST_STREAM_MODE> cfg_simulcast_stream_mode;
    bool support_higher_standard_bitrate;
    VideoQoEAnalyzerParameter qoe_analyzer_parameters;
  };

  struct DetachInfo {
    VideoNodeRtpSink* network;
    DetachReason reason;
  };

  ILocalVideoTrackEx() : id_(id_generator_++) {}
  virtual ~ILocalVideoTrackEx() {}

  virtual bool hasPublished() = 0;

  virtual int setVideoEncoderConfigurationEx(const VideoEncoderConfiguration& config, utils::ConfigPriority priority = utils::CONFIG_PRIORITY_USER) = 0;

  virtual int SetVideoConfigEx(const VideoConfigurationEx& configEx, utils::ConfigPriority priority = utils::CONFIG_PRIORITY_USER) = 0;

  virtual int ResetVideoConfigExByPriority(utils::ConfigPriority priority) = 0;

  virtual int GetConfigExs(std::vector<VideoConfigurationEx>& configs, bool include_disable_config = false) = 0;

  virtual void AddVideoAvailabilityIndicatorEvents(VideoAvailabilityIndicator event) {}

  virtual void GetVideoAvailabilityIndicatorEvents(std::vector<VideoAvailabilityIndicator>& events) {}

  virtual int setUserId(uid_t uid) { user_id_ = uid; return 0; }

  virtual uid_t getUserId() { return user_id_; }

  virtual int GetActiveStreamsCount() = 0;

  virtual int prepareNodes(const char* id = nullptr) = 0;

  virtual bool attach(const AttachInfo& info) = 0;
  virtual bool detach(const DetachInfo& info) = 0;
  virtual bool registerTrackObserver(std::shared_ptr<IVideoTrackObserver> observer) {
    return false;
  }
  virtual bool unregisterTrackObserver(IVideoTrackObserver* observer) {
    return false;
  }

  virtual bool getStatisticsEx(LocalVideoTrackStatsEx& statsEx) { return false; }
  virtual int32_t Width() const = 0;
  virtual int32_t Height() const = 0;
  virtual void getBillingVideoProfile(int32_t& w, int32_t& h) {};
  virtual bool Enabled() const = 0;
  // TODO(Qingyou Pan): Need refine code to remove this interface.
  virtual int addVideoWatermark(const char* watermarkUrl, const WatermarkOptions& options) { return -ERR_NOT_SUPPORTED; }
  virtual int clearVideoWatermarks() { return -ERR_NOT_SUPPORTED; }

  virtual VideoTrackConfigurator* GetVideoTrackConfigurator() {
    return nullptr;
  }

  virtual InternalVideoSourceType getInternalVideoSourceType() { return InternalVideoSourceType::None; }

  virtual rtc::VideoEncoderConfiguration getVideoEncoderConfiguration() { return {}; }

  virtual void getSimucastStreamConfig(SimulcastStreamConfig& simu_stream_config) {}
  virtual void getSimucastStreamStatus(SIMULCAST_STREAM_MODE& mode, bool& enable) {}

  virtual int updateContentHint(VIDEO_CONTENT_HINT contentHint) { return -ERR_NOT_SUPPORTED; }

  virtual int updateScreenCaptureScenario(SCREEN_SCENARIO_TYPE screenScenario) { return -ERR_NOT_SUPPORTED; }

  int TrackId() const { return id_; }

  void setUniqueId(const std::string& unique_id) { unique_id_ = unique_id; }

  virtual int registerVideoEncodedImageReceiver(media::IVideoEncodedFrameObserver* videoReceiver) {return -ERR_NOT_SUPPORTED;}

  virtual int unregisterVideoEncodedImageReceiver(media::IVideoEncodedFrameObserver* videoReceiver) {return -ERR_NOT_SUPPORTED;}

  virtual int setLocalVideoSend(bool send) = 0;

  virtual int getEncoderType() = 0;
  
  virtual int setVideoDumpMode(int mode, bool enabled) { return 0; }

  virtual bool onNegotiationCodecChange(agora::rtc::VIDEO_CODEC_TYPE codec_type,
                                uint8_t negotiated_video_decode_caps) { return false; }
  
  virtual int onRequestEnableSimulcastStream() { return 0; }

  virtual bool getIsAttachedToNetwork() { return false; }

  virtual bool isVideoFilterEnabled(const char* id) { return false; }

  virtual void ReconfigureFecMethod(int fec_method, int dmec_version, int fec_mul_rdc) {}

  virtual bool setEncoderTemporlayers(int temporlayersNum) { return false; }

  virtual bool setH264BframeNumber(int bframeNum)  { return false; }

  virtual bool enableMinorStreamPeriodicKeyFrame()  { return false; }

  virtual void registerProactiveCaller(const std::shared_ptr<rtc::ProactiveCaller>& configurator) { return; }
  virtual void unregisterProactiveCaller() { return; }

  virtual void registerModuleControlPanel(std::shared_ptr<rtc::IModuleControlPanel> panel) { return; }
  virtual void unregisterModuleControlPanel() { return; }
 protected:
  const int id_;
  utils::WeakObserversFacility<IVideoTrackObserver> track_observers_;
  uid_t user_id_;
  std::string unique_id_;

 private:
  static std::atomic<int> id_generator_;
};

struct RemoteVideoTrackStatsEx : RemoteVideoTrackStats {
  uint64_t firstDecodingTimeTickMs = 0;
  uint64_t firstVideoFrameRendered = 0;
  bool isHardwareCodec = false;
  int64_t totalFrozen200ms = 0;
  uint32_t last_frame_max = 0;
  uint32_t dec_in_num = 0;
  uint32_t render_in_num = 0;
  uint32_t render_out_num = 0;
  uint32_t fec_pkts_num = 0;
  uint32_t loss_af_fec = 0;
  int jitter_offset_ms = 0;
  int decode_level[10] = {0};
  uint64_t qp_sum = 0;
  VideoContentType content_type = VideoContentType::UNSPECIFIED;
  std::vector<VideoAvailabilityIndicator> video_availability;
};

class IRemoteVideoTrackEx : public IRemoteVideoTrack,
                            public VideoRemoteTrackControlAspect {
 public:
  enum DetachReason { MANUAL, TRACK_DESTROY, NETWORK_DESTROY };
  using RemoteVideoEvents = StateEvents<REMOTE_VIDEO_STATE, REMOTE_VIDEO_STATE_REASON>;

  struct AttachInfo {
    VideoNodeRtpSource* source;
    VideoNodeRtpSink* rtcp_sender;
    WeakPipelineBuilder builder;
    bool recv_media_packet = false;
    uint64_t stats_space = 0;
    webrtc::ISmoothRender* smooth_render = nullptr;
    bool disable_rewrite_num_reorder_frame = false;
    std::shared_ptr<webrtc::IRsfecCodecFactoryInterface> rsfec_codec_factory;
    uint32_t video_threshhold_ms = 0;
    VideoQoEAnalyzerParameter qoe_analyzer_parameters;
  };

  struct DetachInfo {
    VideoNodeRtpSource* source;
    VideoNodeRtpSink* rtcp_sender;
    DetachReason reason;
  };

  IRemoteVideoTrackEx() = default;

  virtual ~IRemoteVideoTrackEx() {}

  virtual uint32_t getRemoteSsrc() = 0;

  virtual bool attach(const AttachInfo& info, REMOTE_VIDEO_STATE_REASON reason) = 0;
  virtual bool detach(const DetachInfo& info, REMOTE_VIDEO_STATE_REASON reason) = 0;

  virtual bool getStatisticsEx(RemoteVideoTrackStatsEx& statsex) { return false; }

  virtual bool registerTrackObserver(std::shared_ptr<IVideoTrackObserver> observer) {
    return false;
  }
  virtual bool unregisterTrackObserver(IVideoTrackObserver* observer) {
    return false;
  }
  virtual void registerProactiveCaller(const std::shared_ptr<ProactiveCaller>&) {}
  virtual void unregisterProactiveCaller() {}

 protected:
  utils::WeakObserversFacility<IVideoTrackObserver> track_observers_;
};

}  // namespace rtc
}  // namespace agora
