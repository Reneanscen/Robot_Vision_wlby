//
//  Agora Media SDK
//
//  Created by Yaqi Li in 2021-12.
//  Copyright (c) 2021 Agora IO. All rights reserved.
//
#pragma once

namespace agora {
namespace extension {

static const char* const SCAP_PROP_KEY_CAP_USE_MGF = "cap_use_mgf";
static const char* const SCAP_PROP_KEY_CAP_USE_DXGI = "cap_use_dxgi";
static const char* const SCAP_PROP_KEY_CAP_USE_WGC = "cap_use_wgc";
static const char* const SCAP_PROP_KEY_CAP_FORCE_SCREEN = "cap_force_screen";
static const char* const SCAP_PROP_KEY_CAP_WINDOW_FOCUS = "cap_window_focus";
static const char* const SCAP_PROP_KEY_CAP_IMPL_TYPE = "cap_impl_type";
static const char* const SCAP_PROP_KEY_CAP_COLOR_MATRIX = "cap_color_matrix";
static const char* const SCAP_PROP_KEY_CAP_COLOR_RANGE = "cap_color_range";
static const char* const SCAP_PROP_KEY_CAP_MODE = "cap_mode";

static const char* const SCAP_PROP_VAL_CAP_IMPL_MAGNIFY = "cap_impl_magnification";
static const char* const SCAP_PROP_VAL_CAP_IMPL_DXGI = "cap_impl_dxgi";
static const char* const SCAP_PROP_VAL_CAP_IMPL_GDI = "cap_impl_gdi";
static const char* const SCAP_PROP_VAL_CAP_IMPL_AUTO = "cap_impl_auto";

static const char* const SCAP_EVENT_WINDOW_CLOSED = "cap_window_closed";
static const char* const SCAP_EVENT_WINDOW_MINIMIZED = "cap_window_minimized";
static const char* const SCAP_EVENT_NO_PERMISION = "cap_no_permision";
static const char* const SCAP_EVENT_OK = "cap_ok";
static const char* const SCAP_EVENT_CAPTURE_CONNECTED = "cap_connected";
static const char* const SCAP_EVENT_CAPTURE_DISCONNECTED = "cap_disconnected";
static const char* const SCAP_EVENT_CAPTURE_FAILED = "cap_failed";

static const char* const SCAP_PROP_KEY_IPC_PORT = "cap_ipc_port";
static const char* const SCAP_PROP_KEY_CAP_AUDIO = "cap_audio";
static const char* const SCAP_PROP_KEY_CAP_VIDEO = "cap_video";
static const char* const SCAP_PROP_KEY_CAP_MAX_AUDIO_FRAME = "cap_max_audio_frame";

static const char* const SCAP_PROP_KEY_CAP_CROP_WIN = "cap_crop_window";
static const char* const SCAP_PROP_KEY_CAP_MASK_OCCLUED = "cap_maskocclued_window";
static const char* const SCAP_PROP_KEY_CAP_MUTI_GPU = "cap_mutigpu_exclude";

} // namespace extension
} // namespace agora
