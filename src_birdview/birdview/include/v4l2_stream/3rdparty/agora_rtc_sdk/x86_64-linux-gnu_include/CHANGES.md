Note: Please update this file for every Agora API change you do. Simply fill in
your updates in the Working section below.


Agora Low Level APIs (Working)
==================================================

API (yyyy-mm-dd)
==================================================
Purpose of this change

API file name #1
-------------
**Add:**
Short description
- Foo()
- Bar()

**Modified:**
Short description
- Changes Foo() to Foo1()
- Changes Bar() to Bar1()

**Deleted:**
Short description
- Deleted Foo()

API file name #2

API (2023-05-05)
==================================================
deprecate api in IAgoraMediaPlayer.h

IAgoraMediaPlayer.h
-------------
**Modified:**
- getPlayerSdkVersion deprecated

API (2023-04-19)
==================================================
IAgoraMediaRecorder.h
-------------
**Modified:**
- Modify startRecording API comments

API (2023-02-21)
==================================================
IAgoraMediaPlayer.h
-------------
**Add:**
- Add __deprecated tag

API (2021-1-31)
==================================================
IAgoraMediaRecorder.h
-------------
**Modified:**
- setMediaRecorderObserver delete param connection
- startRecording delete param connection
- stopRecording delete param connection

**Deleted:**
- IMediaRecorder's release

API (2021-10-28)
==================================================
IAgoraMediaPlayer.h
-------------
**Modified:**
- rename agora::media::base::IAudioFrameObserver to agora::media::IAudioPcmFrameSink

API (2022-5-13)
==================================================
Add methods for control to follow default audio device

IAudioDeviceManager.h
-------------
**Add:**
- int setLoopbackDevice(const char deviceId[MAX_DEVICE_ID_LENGTH]) = 0;
- int getLoopbackDevice(char deviceId[MAX_DEVICE_ID_LENGTH]) = 0;

API (2022-06-30)
==================================================
IAgoraMediaPlayer.h
-------------
**Deleted:**
- IMediaPlayer's muteAudio & isAudioMuted & muteVideo & isVideoMuted

API (2022-4-18)
==================================================
Add methods for control to follow default audio device

IAudioDeviceManager.h
-------------
**Add:**
- int followSystemPlaybackDevice(bool enable) = 0;
- int followSystemRecordingDevice(bool enable) = 0;

API (2022-04-14)
==================================================
marked deprecated
virtual int openWithCustomSource(int64_t startPos, media::base::IMediaPlayerCustomDataProvider* provider) 

API (2022-4-8)
==================================================
Add methods for Agora media player

IAgoraMediaPlayer.h
-------------
**Add:**
- int setSoundPositionParams(float pan, float gain);

API (2022-04-01)
==================================================
IAudioDeviceManager.h
-------------
**Add:**
- int getDefaultDevice(char deviceName[MAX_DEVICE_ID_LENGTH], char deviceId[MAX_DEVICE_ID_LENGTH]);

API (2022-02-27)
==================================================
Modify arguments name

API (2021-12-25)
==================================================
Add methods for Agora media player

IAgoraMediaPlayer.h
-------------
**Modified:**
update comment

API (2021-12-25)
==================================================
Add methods for Agora media player

IAgoraMediaPlayer.h
-------------
**Add:**
- int setSpatialAudioParams(const SpatialAudioParams&  spatial_audio_params)

API (2022-01-24)
==================================================
add function: play and cache at the same time
- virtual int openWithMediaSource(const media::base::MediaSource &source) = 0;
- Class IMediaPlayerCacheManager
- agora::rtc::IMediaPlayerCacheManager* AGORA_CALL getMediaPlayerCacheManager();
 
API (2021-11-01)
==================================================
add new api in IAgoraMediaPlayer.h

IAgoraMediaPlayer.h
-------------
**Add:**
-  virtual int unloadSrc(const char* src) = 0;
-------------
**Modify:**
-  reanme changePlaybackSpeed to setPlaybackSpeed

API (2021-09-08)
==================================================
IAgoraRhythmPlayer.h
-------------
Add IRhythmPlayer interface class

API (2021-10-09)
==================================================
add new api in IAgoraMediaPlayer.h

IAgoraMediaPlayer.h
-------------
**Add:**
- int openWithAgoraCDNSrc(const char* src, int64_t startPos)
- int getAgoraCDNLineCount()
- int switchAgoraCDNLineByIndex(int index)
- int getCurrentAgoraCDNIndex()
- int enableAutoSwitchAgoraCDN(bool enable)
- int renewAgoraCDNSrcToken(const char* token, int64_t ts)
- int switchAgoraCDNSrc(const char* src, bool syncPts = false) 
- int switchSrc(const char* src, bool syncPts) 
- int preloadSrc(const char* src, int64_t startPos)
- int playPreloadedSrc(const char* src)
 

API (2021-07-15)
==================================================

IAgoraMediaPlayer.h
-------------
**Add:**
- setAudioDualMonoMode

API (2021-07-02)
==================================================
Add methods for Agora media player

IAgoraMediaPlayer.h
-------------
**Add:**
- setAudioPitch(int pitch)

API (2021-07-15)
==================================================
API (2021-06-18)
==================================================
Add methods for Agora media player

IAgoraMediaPlayer.h
-------------
**Add:**
- registerAudioFrameObserver(media::base::IAudioFrameObserver* observer, RAW_AUDIO_FRAME_OP_MODE_TYPE mode)
- add method muteAudio()
- add method isAudioMuted()
- add method muteVideo()
- add method isVideoMuted()
- add method registerMediaPlayerAudioSpectrumObserver(media::IAudioSpectrumObserver* observer,int intervalInMS)  
- add method unregisterMediaPlayerAudioSpectrumObserver(media::IAudioSpectrumObserver* observer)  API (2021-06-15)
==================================================
Add new api file IAudioDeviceManager.h.

API (2021-05-31)
==================================================
Add new api file IAgoraMediaComponentFactory.h.

API (2021-06-18)
==================================================
add api in IAgoraMediaPlayer.h

IAgoraMediaPlayer.h
-------------
**Add:**
- getPlaySrc

API (2022-02-22)
==================================================
refine api in IAgoraMediaPlayer.h

IAgoraMediaPlayer.h
-------------
**Modified:**
- registerMediaPlayerAudioSpectrumObserver
- unregisterMediaPlayerAudioSpectrumObserver

API (2020-04-01)
==================================================
add api in IAgoraMediaPlayer.h

IAgoraMediaPlayer.h
-------------
**Add:**
- setPlayerOption
- getPlayerSdkVersion

API (2020-03-19)
==================================================
modify to adapt c++ 11

Update struct name (2021-01-21)
==================================================
IAgoraMediaPlayer.h
-------------
Change name of MediaStreamInfo to PlayerStreamInfo.


API (2020-11-13)
==================================================
Add IAgoraMediaPlayer.h

API (2021-0603-13)
==================================================
Add IAudioDeviceManager.h
==================================================

API (2022-05-10)
==================================================
**Add:**
-------------
* IAgoraRtcEngineEx.h
Add Class for Media Recorder
 -add Class IMediaRecorder
