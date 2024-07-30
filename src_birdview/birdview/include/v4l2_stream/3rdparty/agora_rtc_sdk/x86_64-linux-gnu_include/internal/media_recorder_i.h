//
//  media_recorder.hpp
//
//  Created by zexiong qin on 2019-06.
//  Copyright © 2018 Agora. All rights reserved.
//

#pragma once

#include <stdint.h>

#include "IAgoraRtcEngine.h"
#include "IAgoraMediaEngine.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef void AyseMuxerContext;

#define AYSE_IO_FLAG_READ 0x1
#define AYSE_IO_FLAG_WRITE 0x2
#define AYSE_IO_FLAG_ODIRECT 0x4

#ifdef __cplusplus
}
#endif

namespace agora {
namespace media {
class IMediaRecorderObserverEx {
 public:
  /**
   * Occurs when the recording state changes.
   *
   * @since v4.0.0
   *
   * When the local audio and video recording state changes, the SDK triggers this callback to report the current
   * recording state and the reason for the change.
   *
   * @param state The current recording state. See \ref agora::media::RecorderState "RecorderState".
   * @param error The reason for the state change. See \ref agora::media::RecorderErrorCode "RecorderErrorCode".
   */
  virtual void onRecorderStateChanged(RecorderState state, RecorderErrorCode error) = 0;
  /**
   * Occurs when the recording information is updated.
   *
   * @since v4.0.0
   *
   * After you successfully register this callback and enable the local audio and video recording, the SDK periodically triggers
   * the `onRecorderInfoUpdated` callback based on the set value of `recorderInfoUpdateInterval`. This callback reports the
   * filename, duration, and size of the current recording file.
   *
   * @param info Information for the recording file. See RecorderInfo.
   *
   */
  virtual void onRecorderInfoUpdated(const RecorderInfo& info) = 0;
  virtual ~IMediaRecorderObserverEx() {}
};
}
namespace rtc {

/** Definition of AVDataObserver
 */
class IAVDataObserver {
 public:
  /** Metadata type of the observer.
   @note We only support video metadata for now.
   */
  enum AVDATA_TYPE {
    /** 0: the metadata type is unknown.
     */
    AVDATA_UNKNOWN = 0,
    /** 1: the metadata type is video.
     */
    AVDATA_VIDEO = 1,
    /** 2: the metadata type is video.
     */
    AVDATA_AUDIO = 2,
    /** 2: the metadata type is video.
     */
    AVDATA_AUDIO_MUTE = 3,
  };

  enum CODEC_VIDEO {
    /** 0: h264 avc codec.
     */
    CODEC_VIDEO_AVC = 0,
    /** 1: h265 hevc codec.
     */
    CODEC_VIDEO_HEVC = 1,
    /** 2: vp8 codec.
     */
    CODEC_VIDEO_VP8 = 2,
  };

  enum CODEC_AUDIO {
    /** 0: PCM audio codec.
     */
    CODEC_AUDIO_PCM = 0,
    /** 1: aac audio codec.
     */
    CODEC_AUDIO_AAC = 1,
    /** 2: G711 audio codec.
     */
    CODEC_AUDIO_G722 = 2,
  };

  struct VDataInfo {
    unsigned int codec;
    unsigned int width;
    unsigned int height;
    int frameType;
    int rotation;
    bool equal(const VDataInfo& vinfo) const {
      return codec == vinfo.codec && width == vinfo.width && height == vinfo.height &&
             rotation == vinfo.rotation;
    }
  };

  struct ADataInfo {
    unsigned int codec;
    unsigned int bitwidth;
    unsigned int sample_rate;
    unsigned int channel;
    unsigned int sample_size;

    bool equal(const ADataInfo& ainfo) const {
      return codec == ainfo.codec && bitwidth == ainfo.bitwidth &&
             sample_rate == ainfo.sample_rate && channel == ainfo.channel;
    };
  };

  struct AVData {
    /** The User ID. reserved
     - For the receiver: the ID of the user who owns the data.
     */
    unsigned int uid;
    /**
     - data type, audio / video.
     */
    enum AVDATA_TYPE type;
    /** Buffer size of the sent or received Metadata.
     */
    unsigned int size;
    /** Buffer address of the sent or received Metadata.
     */
    unsigned char* buffer;
    /** Time statmp of the frame following the metadata.
     */
    unsigned int timestamp;
    /**
     * Video frame info
     */
    VDataInfo vinfo;
    /**
     * Audio frame info
     */
    ADataInfo ainfo;
  };

  virtual ~IAVDataObserver(){};

  /** Occurs when audio/video data ready.

   param avdata The received Metadata.
   */
  virtual bool onAVDataReady(const AVData& avdata) = 0;
}; 

class IMediaRecorderEX : public IAVDataObserver, public agora::media::IAudioFrameObserver, public agora::media::IVideoEncodedFrameObserver {
 public:

  virtual int startRecording(const media::MediaRecorderConfiguration& config) = 0;

  virtual int stopRecording() = 0;

  virtual void release() = 0;

  virtual void setMediaRecorderObserver(media::IMediaRecorderObserverEx* observer) = 0;

  virtual void setSysVersion(int sys_version) = 0;

  virtual void setRemoteUid(uid_t uid) = 0;

  bool onPlaybackAudioFrame(const char* channelId, AudioFrame& audioFrame) override{return true;}
  bool onMixedAudioFrame(const char* channelId, AudioFrame& audioFrame) override{return true;}
  bool onEarMonitoringAudioFrame(AudioFrame& audioFrame) override { return true; }
  bool onPlaybackAudioFrameBeforeMixing(const char* channelId, user_id_t userId, AudioFrame& audioFrame) override {
    return true;
  }
  bool onPlaybackAudioFrameBeforeMixing(const char* channelId, rtc::uid_t uid, AudioFrame& audioFrame) override {
    return true;
  }

  AudioParams getPlaybackAudioParams() override { return AudioParams(48000, 1, RAW_AUDIO_FRAME_OP_MODE_READ_ONLY, 480); }

  AudioParams getRecordAudioParams() override { return AudioParams(48000, 1, RAW_AUDIO_FRAME_OP_MODE_READ_ONLY, 480); }

  AudioParams getMixedAudioParams() override { return AudioParams(48000, 1, RAW_AUDIO_FRAME_OP_MODE_READ_ONLY, 480); }

  AudioParams getEarMonitoringAudioParams() override{ return AudioParams(); }

  virtual bool onEncodedVideoFrameReceived(rtc::uid_t uid, const uint8_t* imageBuffer, size_t length,
                                   const rtc::EncodedVideoFrameInfo& videoEncodedFrameInfo) override {
    IAVDataObserver::AVData avData;
    avData.type = AVDATA_VIDEO;
    avData.size = length;
    avData.buffer = (unsigned char *)imageBuffer;
    avData.vinfo.codec = videoEncodedFrameInfo.codecType;
    avData.vinfo.width = videoEncodedFrameInfo.width;
    avData.vinfo.height = videoEncodedFrameInfo.height;
    avData.vinfo.frameType = videoEncodedFrameInfo.frameType;
    avData.vinfo.rotation = videoEncodedFrameInfo.rotation;
    return onAVDataReady(avData);
  }
};

}  // namespace rtc
}  // namespace agora
