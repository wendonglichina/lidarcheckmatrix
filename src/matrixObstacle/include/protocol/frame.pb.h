// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: frame.proto

#ifndef PROTOBUF_frame_2eproto__INCLUDED
#define PROTOBUF_frame_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "common.pb.h"
#include "frame.v0.pb.h"
#include "frame.v1.pb.h"
// @@protoc_insertion_point(includes)

namespace FrameProtocol {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_frame_2eproto();
void protobuf_AssignDesc_frame_2eproto();
void protobuf_ShutdownFile_frame_2eproto();

class Frame;
class FrameShuffle;

enum FrameProtoVersion {
  FrameProtoVersion_0 = 0,
  FrameProtoVersion_1 = 1
};
bool FrameProtoVersion_IsValid(int value);
const FrameProtoVersion FrameProtoVersion_MIN = FrameProtoVersion_0;
const FrameProtoVersion FrameProtoVersion_MAX = FrameProtoVersion_1;
const int FrameProtoVersion_ARRAYSIZE = FrameProtoVersion_MAX + 1;

const ::google::protobuf::EnumDescriptor* FrameProtoVersion_descriptor();
inline const ::std::string& FrameProtoVersion_Name(FrameProtoVersion value) {
  return ::google::protobuf::internal::NameOfEnum(
    FrameProtoVersion_descriptor(), value);
}
inline bool FrameProtoVersion_Parse(
    const ::std::string& name, FrameProtoVersion* value) {
  return ::google::protobuf::internal::ParseNamedEnum<FrameProtoVersion>(
    FrameProtoVersion_descriptor(), name, value);
}
// ===================================================================

class Frame : public ::google::protobuf::Message {
 public:
  Frame();
  virtual ~Frame();

  Frame(const Frame& from);

  inline Frame& operator=(const Frame& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Frame& default_instance();

  void Swap(Frame* other);

  // implements Message ----------------------------------------------

  Frame* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const Frame& from);
  void MergeFrom(const Frame& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 version = 1;
  inline bool has_version() const;
  inline void clear_version();
  static const int kVersionFieldNumber = 1;
  inline ::google::protobuf::int32 version() const;
  inline void set_version(::google::protobuf::int32 value);

  // required int32 frame_id = 2;
  inline bool has_frame_id() const;
  inline void clear_frame_id();
  static const int kFrameIdFieldNumber = 2;
  inline ::google::protobuf::int32 frame_id() const;
  inline void set_frame_id(::google::protobuf::int32 value);

  // optional .CommonProto.CameraMatrix camera = 3;
  inline bool has_camera() const;
  inline void clear_camera();
  static const int kCameraFieldNumber = 3;
  inline const ::CommonProto::CameraMatrix& camera() const;
  inline ::CommonProto::CameraMatrix* mutable_camera();
  inline ::CommonProto::CameraMatrix* release_camera();
  inline void set_allocated_camera(::CommonProto::CameraMatrix* camera);

  // optional .CommonProto.Image img_frame = 4;
  inline bool has_img_frame() const;
  inline void clear_img_frame();
  static const int kImgFrameFieldNumber = 4;
  inline const ::CommonProto::Image& img_frame() const;
  inline ::CommonProto::Image* mutable_img_frame();
  inline ::CommonProto::Image* release_img_frame();
  inline void set_allocated_img_frame(::CommonProto::Image* img_frame);

  // optional int32 proto_version = 5 [default = 1];
  inline bool has_proto_version() const;
  inline void clear_proto_version();
  static const int kProtoVersionFieldNumber = 5;
  inline ::google::protobuf::int32 proto_version() const;
  inline void set_proto_version(::google::protobuf::int32 value);

  // optional .FrameV0Proto.Frame frame_v0 = 6;
  inline bool has_frame_v0() const;
  inline void clear_frame_v0();
  static const int kFrameV0FieldNumber = 6;
  inline const ::FrameV0Proto::Frame& frame_v0() const;
  inline ::FrameV0Proto::Frame* mutable_frame_v0();
  inline ::FrameV0Proto::Frame* release_frame_v0();
  inline void set_allocated_frame_v0(::FrameV0Proto::Frame* frame_v0);

  // optional .FrameV1Proto.Frame frame_v1 = 7;
  inline bool has_frame_v1() const;
  inline void clear_frame_v1();
  static const int kFrameV1FieldNumber = 7;
  inline const ::FrameV1Proto::Frame& frame_v1() const;
  inline ::FrameV1Proto::Frame* mutable_frame_v1();
  inline ::FrameV1Proto::Frame* release_frame_v1();
  inline void set_allocated_frame_v1(::FrameV1Proto::Frame* frame_v1);

  // @@protoc_insertion_point(class_scope:FrameProtocol.Frame)
 private:
  inline void set_has_version();
  inline void clear_has_version();
  inline void set_has_frame_id();
  inline void clear_has_frame_id();
  inline void set_has_camera();
  inline void clear_has_camera();
  inline void set_has_img_frame();
  inline void clear_has_img_frame();
  inline void set_has_proto_version();
  inline void clear_has_proto_version();
  inline void set_has_frame_v0();
  inline void clear_has_frame_v0();
  inline void set_has_frame_v1();
  inline void clear_has_frame_v1();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int32 version_;
  ::google::protobuf::int32 frame_id_;
  ::CommonProto::CameraMatrix* camera_;
  ::CommonProto::Image* img_frame_;
  ::FrameV0Proto::Frame* frame_v0_;
  ::FrameV1Proto::Frame* frame_v1_;
  ::google::protobuf::int32 proto_version_;
  friend void  protobuf_AddDesc_frame_2eproto();
  friend void protobuf_AssignDesc_frame_2eproto();
  friend void protobuf_ShutdownFile_frame_2eproto();

  void InitAsDefaultInstance();
  static Frame* default_instance_;
};
// -------------------------------------------------------------------

class FrameShuffle : public ::google::protobuf::Message {
 public:
  FrameShuffle();
  virtual ~FrameShuffle();

  FrameShuffle(const FrameShuffle& from);

  inline FrameShuffle& operator=(const FrameShuffle& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const FrameShuffle& default_instance();

  void Swap(FrameShuffle* other);

  // implements Message ----------------------------------------------

  FrameShuffle* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const FrameShuffle& from);
  void MergeFrom(const FrameShuffle& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 version = 1;
  inline bool has_version() const;
  inline void clear_version();
  static const int kVersionFieldNumber = 1;
  inline ::google::protobuf::int32 version() const;
  inline void set_version(::google::protobuf::int32 value);

  // required int32 frame_id = 2;
  inline bool has_frame_id() const;
  inline void clear_frame_id();
  static const int kFrameIdFieldNumber = 2;
  inline ::google::protobuf::int32 frame_id() const;
  inline void set_frame_id(::google::protobuf::int32 value);

  // optional .CommonProto.CameraMatrix camera = 3;
  inline bool has_camera() const;
  inline void clear_camera();
  static const int kCameraFieldNumber = 3;
  inline const ::CommonProto::CameraMatrix& camera() const;
  inline ::CommonProto::CameraMatrix* mutable_camera();
  inline ::CommonProto::CameraMatrix* release_camera();
  inline void set_allocated_camera(::CommonProto::CameraMatrix* camera);

  // optional .CommonProto.Image img_frame = 4;
  inline bool has_img_frame() const;
  inline void clear_img_frame();
  static const int kImgFrameFieldNumber = 4;
  inline const ::CommonProto::Image& img_frame() const;
  inline ::CommonProto::Image* mutable_img_frame();
  inline ::CommonProto::Image* release_img_frame();
  inline void set_allocated_img_frame(::CommonProto::Image* img_frame);

  // optional int32 proto_version = 5 [default = 1];
  inline bool has_proto_version() const;
  inline void clear_proto_version();
  static const int kProtoVersionFieldNumber = 5;
  inline ::google::protobuf::int32 proto_version() const;
  inline void set_proto_version(::google::protobuf::int32 value);

  // optional .FrameV0Proto.FrameShuffle frame_v0 = 6;
  inline bool has_frame_v0() const;
  inline void clear_frame_v0();
  static const int kFrameV0FieldNumber = 6;
  inline const ::FrameV0Proto::FrameShuffle& frame_v0() const;
  inline ::FrameV0Proto::FrameShuffle* mutable_frame_v0();
  inline ::FrameV0Proto::FrameShuffle* release_frame_v0();
  inline void set_allocated_frame_v0(::FrameV0Proto::FrameShuffle* frame_v0);

  // optional .FrameV1Proto.FrameShuffle frame_v1 = 7;
  inline bool has_frame_v1() const;
  inline void clear_frame_v1();
  static const int kFrameV1FieldNumber = 7;
  inline const ::FrameV1Proto::FrameShuffle& frame_v1() const;
  inline ::FrameV1Proto::FrameShuffle* mutable_frame_v1();
  inline ::FrameV1Proto::FrameShuffle* release_frame_v1();
  inline void set_allocated_frame_v1(::FrameV1Proto::FrameShuffle* frame_v1);

  // @@protoc_insertion_point(class_scope:FrameProtocol.FrameShuffle)
 private:
  inline void set_has_version();
  inline void clear_has_version();
  inline void set_has_frame_id();
  inline void clear_has_frame_id();
  inline void set_has_camera();
  inline void clear_has_camera();
  inline void set_has_img_frame();
  inline void clear_has_img_frame();
  inline void set_has_proto_version();
  inline void clear_has_proto_version();
  inline void set_has_frame_v0();
  inline void clear_has_frame_v0();
  inline void set_has_frame_v1();
  inline void clear_has_frame_v1();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::int32 version_;
  ::google::protobuf::int32 frame_id_;
  ::CommonProto::CameraMatrix* camera_;
  ::CommonProto::Image* img_frame_;
  ::FrameV0Proto::FrameShuffle* frame_v0_;
  ::FrameV1Proto::FrameShuffle* frame_v1_;
  ::google::protobuf::int32 proto_version_;
  friend void  protobuf_AddDesc_frame_2eproto();
  friend void protobuf_AssignDesc_frame_2eproto();
  friend void protobuf_ShutdownFile_frame_2eproto();

  void InitAsDefaultInstance();
  static FrameShuffle* default_instance_;
};
// ===================================================================


// ===================================================================

// Frame

// required int32 version = 1;
inline bool Frame::has_version() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Frame::set_has_version() {
  _has_bits_[0] |= 0x00000001u;
}
inline void Frame::clear_has_version() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void Frame::clear_version() {
  version_ = 0;
  clear_has_version();
}
inline ::google::protobuf::int32 Frame::version() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.version)
  return version_;
}
inline void Frame::set_version(::google::protobuf::int32 value) {
  set_has_version();
  version_ = value;
  // @@protoc_insertion_point(field_set:FrameProtocol.Frame.version)
}

// required int32 frame_id = 2;
inline bool Frame::has_frame_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void Frame::set_has_frame_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void Frame::clear_has_frame_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void Frame::clear_frame_id() {
  frame_id_ = 0;
  clear_has_frame_id();
}
inline ::google::protobuf::int32 Frame::frame_id() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.frame_id)
  return frame_id_;
}
inline void Frame::set_frame_id(::google::protobuf::int32 value) {
  set_has_frame_id();
  frame_id_ = value;
  // @@protoc_insertion_point(field_set:FrameProtocol.Frame.frame_id)
}

// optional .CommonProto.CameraMatrix camera = 3;
inline bool Frame::has_camera() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void Frame::set_has_camera() {
  _has_bits_[0] |= 0x00000004u;
}
inline void Frame::clear_has_camera() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void Frame::clear_camera() {
  if (camera_ != NULL) camera_->::CommonProto::CameraMatrix::Clear();
  clear_has_camera();
}
inline const ::CommonProto::CameraMatrix& Frame::camera() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.camera)
  return camera_ != NULL ? *camera_ : *default_instance_->camera_;
}
inline ::CommonProto::CameraMatrix* Frame::mutable_camera() {
  set_has_camera();
  if (camera_ == NULL) camera_ = new ::CommonProto::CameraMatrix;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.Frame.camera)
  return camera_;
}
inline ::CommonProto::CameraMatrix* Frame::release_camera() {
  clear_has_camera();
  ::CommonProto::CameraMatrix* temp = camera_;
  camera_ = NULL;
  return temp;
}
inline void Frame::set_allocated_camera(::CommonProto::CameraMatrix* camera) {
  delete camera_;
  camera_ = camera;
  if (camera) {
    set_has_camera();
  } else {
    clear_has_camera();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.Frame.camera)
}

// optional .CommonProto.Image img_frame = 4;
inline bool Frame::has_img_frame() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void Frame::set_has_img_frame() {
  _has_bits_[0] |= 0x00000008u;
}
inline void Frame::clear_has_img_frame() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void Frame::clear_img_frame() {
  if (img_frame_ != NULL) img_frame_->::CommonProto::Image::Clear();
  clear_has_img_frame();
}
inline const ::CommonProto::Image& Frame::img_frame() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.img_frame)
  return img_frame_ != NULL ? *img_frame_ : *default_instance_->img_frame_;
}
inline ::CommonProto::Image* Frame::mutable_img_frame() {
  set_has_img_frame();
  if (img_frame_ == NULL) img_frame_ = new ::CommonProto::Image;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.Frame.img_frame)
  return img_frame_;
}
inline ::CommonProto::Image* Frame::release_img_frame() {
  clear_has_img_frame();
  ::CommonProto::Image* temp = img_frame_;
  img_frame_ = NULL;
  return temp;
}
inline void Frame::set_allocated_img_frame(::CommonProto::Image* img_frame) {
  delete img_frame_;
  img_frame_ = img_frame;
  if (img_frame) {
    set_has_img_frame();
  } else {
    clear_has_img_frame();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.Frame.img_frame)
}

// optional int32 proto_version = 5 [default = 1];
inline bool Frame::has_proto_version() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void Frame::set_has_proto_version() {
  _has_bits_[0] |= 0x00000010u;
}
inline void Frame::clear_has_proto_version() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void Frame::clear_proto_version() {
  proto_version_ = 1;
  clear_has_proto_version();
}
inline ::google::protobuf::int32 Frame::proto_version() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.proto_version)
  return proto_version_;
}
inline void Frame::set_proto_version(::google::protobuf::int32 value) {
  set_has_proto_version();
  proto_version_ = value;
  // @@protoc_insertion_point(field_set:FrameProtocol.Frame.proto_version)
}

// optional .FrameV0Proto.Frame frame_v0 = 6;
inline bool Frame::has_frame_v0() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void Frame::set_has_frame_v0() {
  _has_bits_[0] |= 0x00000020u;
}
inline void Frame::clear_has_frame_v0() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void Frame::clear_frame_v0() {
  if (frame_v0_ != NULL) frame_v0_->::FrameV0Proto::Frame::Clear();
  clear_has_frame_v0();
}
inline const ::FrameV0Proto::Frame& Frame::frame_v0() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.frame_v0)
  return frame_v0_ != NULL ? *frame_v0_ : *default_instance_->frame_v0_;
}
inline ::FrameV0Proto::Frame* Frame::mutable_frame_v0() {
  set_has_frame_v0();
  if (frame_v0_ == NULL) frame_v0_ = new ::FrameV0Proto::Frame;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.Frame.frame_v0)
  return frame_v0_;
}
inline ::FrameV0Proto::Frame* Frame::release_frame_v0() {
  clear_has_frame_v0();
  ::FrameV0Proto::Frame* temp = frame_v0_;
  frame_v0_ = NULL;
  return temp;
}
inline void Frame::set_allocated_frame_v0(::FrameV0Proto::Frame* frame_v0) {
  delete frame_v0_;
  frame_v0_ = frame_v0;
  if (frame_v0) {
    set_has_frame_v0();
  } else {
    clear_has_frame_v0();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.Frame.frame_v0)
}

// optional .FrameV1Proto.Frame frame_v1 = 7;
inline bool Frame::has_frame_v1() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void Frame::set_has_frame_v1() {
  _has_bits_[0] |= 0x00000040u;
}
inline void Frame::clear_has_frame_v1() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void Frame::clear_frame_v1() {
  if (frame_v1_ != NULL) frame_v1_->::FrameV1Proto::Frame::Clear();
  clear_has_frame_v1();
}
inline const ::FrameV1Proto::Frame& Frame::frame_v1() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.Frame.frame_v1)
  return frame_v1_ != NULL ? *frame_v1_ : *default_instance_->frame_v1_;
}
inline ::FrameV1Proto::Frame* Frame::mutable_frame_v1() {
  set_has_frame_v1();
  if (frame_v1_ == NULL) frame_v1_ = new ::FrameV1Proto::Frame;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.Frame.frame_v1)
  return frame_v1_;
}
inline ::FrameV1Proto::Frame* Frame::release_frame_v1() {
  clear_has_frame_v1();
  ::FrameV1Proto::Frame* temp = frame_v1_;
  frame_v1_ = NULL;
  return temp;
}
inline void Frame::set_allocated_frame_v1(::FrameV1Proto::Frame* frame_v1) {
  delete frame_v1_;
  frame_v1_ = frame_v1;
  if (frame_v1) {
    set_has_frame_v1();
  } else {
    clear_has_frame_v1();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.Frame.frame_v1)
}

// -------------------------------------------------------------------

// FrameShuffle

// required int32 version = 1;
inline bool FrameShuffle::has_version() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void FrameShuffle::set_has_version() {
  _has_bits_[0] |= 0x00000001u;
}
inline void FrameShuffle::clear_has_version() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void FrameShuffle::clear_version() {
  version_ = 0;
  clear_has_version();
}
inline ::google::protobuf::int32 FrameShuffle::version() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.version)
  return version_;
}
inline void FrameShuffle::set_version(::google::protobuf::int32 value) {
  set_has_version();
  version_ = value;
  // @@protoc_insertion_point(field_set:FrameProtocol.FrameShuffle.version)
}

// required int32 frame_id = 2;
inline bool FrameShuffle::has_frame_id() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void FrameShuffle::set_has_frame_id() {
  _has_bits_[0] |= 0x00000002u;
}
inline void FrameShuffle::clear_has_frame_id() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void FrameShuffle::clear_frame_id() {
  frame_id_ = 0;
  clear_has_frame_id();
}
inline ::google::protobuf::int32 FrameShuffle::frame_id() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.frame_id)
  return frame_id_;
}
inline void FrameShuffle::set_frame_id(::google::protobuf::int32 value) {
  set_has_frame_id();
  frame_id_ = value;
  // @@protoc_insertion_point(field_set:FrameProtocol.FrameShuffle.frame_id)
}

// optional .CommonProto.CameraMatrix camera = 3;
inline bool FrameShuffle::has_camera() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void FrameShuffle::set_has_camera() {
  _has_bits_[0] |= 0x00000004u;
}
inline void FrameShuffle::clear_has_camera() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void FrameShuffle::clear_camera() {
  if (camera_ != NULL) camera_->::CommonProto::CameraMatrix::Clear();
  clear_has_camera();
}
inline const ::CommonProto::CameraMatrix& FrameShuffle::camera() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.camera)
  return camera_ != NULL ? *camera_ : *default_instance_->camera_;
}
inline ::CommonProto::CameraMatrix* FrameShuffle::mutable_camera() {
  set_has_camera();
  if (camera_ == NULL) camera_ = new ::CommonProto::CameraMatrix;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.FrameShuffle.camera)
  return camera_;
}
inline ::CommonProto::CameraMatrix* FrameShuffle::release_camera() {
  clear_has_camera();
  ::CommonProto::CameraMatrix* temp = camera_;
  camera_ = NULL;
  return temp;
}
inline void FrameShuffle::set_allocated_camera(::CommonProto::CameraMatrix* camera) {
  delete camera_;
  camera_ = camera;
  if (camera) {
    set_has_camera();
  } else {
    clear_has_camera();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.FrameShuffle.camera)
}

// optional .CommonProto.Image img_frame = 4;
inline bool FrameShuffle::has_img_frame() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void FrameShuffle::set_has_img_frame() {
  _has_bits_[0] |= 0x00000008u;
}
inline void FrameShuffle::clear_has_img_frame() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void FrameShuffle::clear_img_frame() {
  if (img_frame_ != NULL) img_frame_->::CommonProto::Image::Clear();
  clear_has_img_frame();
}
inline const ::CommonProto::Image& FrameShuffle::img_frame() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.img_frame)
  return img_frame_ != NULL ? *img_frame_ : *default_instance_->img_frame_;
}
inline ::CommonProto::Image* FrameShuffle::mutable_img_frame() {
  set_has_img_frame();
  if (img_frame_ == NULL) img_frame_ = new ::CommonProto::Image;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.FrameShuffle.img_frame)
  return img_frame_;
}
inline ::CommonProto::Image* FrameShuffle::release_img_frame() {
  clear_has_img_frame();
  ::CommonProto::Image* temp = img_frame_;
  img_frame_ = NULL;
  return temp;
}
inline void FrameShuffle::set_allocated_img_frame(::CommonProto::Image* img_frame) {
  delete img_frame_;
  img_frame_ = img_frame;
  if (img_frame) {
    set_has_img_frame();
  } else {
    clear_has_img_frame();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.FrameShuffle.img_frame)
}

// optional int32 proto_version = 5 [default = 1];
inline bool FrameShuffle::has_proto_version() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void FrameShuffle::set_has_proto_version() {
  _has_bits_[0] |= 0x00000010u;
}
inline void FrameShuffle::clear_has_proto_version() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void FrameShuffle::clear_proto_version() {
  proto_version_ = 1;
  clear_has_proto_version();
}
inline ::google::protobuf::int32 FrameShuffle::proto_version() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.proto_version)
  return proto_version_;
}
inline void FrameShuffle::set_proto_version(::google::protobuf::int32 value) {
  set_has_proto_version();
  proto_version_ = value;
  // @@protoc_insertion_point(field_set:FrameProtocol.FrameShuffle.proto_version)
}

// optional .FrameV0Proto.FrameShuffle frame_v0 = 6;
inline bool FrameShuffle::has_frame_v0() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void FrameShuffle::set_has_frame_v0() {
  _has_bits_[0] |= 0x00000020u;
}
inline void FrameShuffle::clear_has_frame_v0() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void FrameShuffle::clear_frame_v0() {
  if (frame_v0_ != NULL) frame_v0_->::FrameV0Proto::FrameShuffle::Clear();
  clear_has_frame_v0();
}
inline const ::FrameV0Proto::FrameShuffle& FrameShuffle::frame_v0() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.frame_v0)
  return frame_v0_ != NULL ? *frame_v0_ : *default_instance_->frame_v0_;
}
inline ::FrameV0Proto::FrameShuffle* FrameShuffle::mutable_frame_v0() {
  set_has_frame_v0();
  if (frame_v0_ == NULL) frame_v0_ = new ::FrameV0Proto::FrameShuffle;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.FrameShuffle.frame_v0)
  return frame_v0_;
}
inline ::FrameV0Proto::FrameShuffle* FrameShuffle::release_frame_v0() {
  clear_has_frame_v0();
  ::FrameV0Proto::FrameShuffle* temp = frame_v0_;
  frame_v0_ = NULL;
  return temp;
}
inline void FrameShuffle::set_allocated_frame_v0(::FrameV0Proto::FrameShuffle* frame_v0) {
  delete frame_v0_;
  frame_v0_ = frame_v0;
  if (frame_v0) {
    set_has_frame_v0();
  } else {
    clear_has_frame_v0();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.FrameShuffle.frame_v0)
}

// optional .FrameV1Proto.FrameShuffle frame_v1 = 7;
inline bool FrameShuffle::has_frame_v1() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void FrameShuffle::set_has_frame_v1() {
  _has_bits_[0] |= 0x00000040u;
}
inline void FrameShuffle::clear_has_frame_v1() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void FrameShuffle::clear_frame_v1() {
  if (frame_v1_ != NULL) frame_v1_->::FrameV1Proto::FrameShuffle::Clear();
  clear_has_frame_v1();
}
inline const ::FrameV1Proto::FrameShuffle& FrameShuffle::frame_v1() const {
  // @@protoc_insertion_point(field_get:FrameProtocol.FrameShuffle.frame_v1)
  return frame_v1_ != NULL ? *frame_v1_ : *default_instance_->frame_v1_;
}
inline ::FrameV1Proto::FrameShuffle* FrameShuffle::mutable_frame_v1() {
  set_has_frame_v1();
  if (frame_v1_ == NULL) frame_v1_ = new ::FrameV1Proto::FrameShuffle;
  // @@protoc_insertion_point(field_mutable:FrameProtocol.FrameShuffle.frame_v1)
  return frame_v1_;
}
inline ::FrameV1Proto::FrameShuffle* FrameShuffle::release_frame_v1() {
  clear_has_frame_v1();
  ::FrameV1Proto::FrameShuffle* temp = frame_v1_;
  frame_v1_ = NULL;
  return temp;
}
inline void FrameShuffle::set_allocated_frame_v1(::FrameV1Proto::FrameShuffle* frame_v1) {
  delete frame_v1_;
  frame_v1_ = frame_v1;
  if (frame_v1) {
    set_has_frame_v1();
  } else {
    clear_has_frame_v1();
  }
  // @@protoc_insertion_point(field_set_allocated:FrameProtocol.FrameShuffle.frame_v1)
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace FrameProtocol

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::FrameProtocol::FrameProtoVersion> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::FrameProtocol::FrameProtoVersion>() {
  return ::FrameProtocol::FrameProtoVersion_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_frame_2eproto__INCLUDED