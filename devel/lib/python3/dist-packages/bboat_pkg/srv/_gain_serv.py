# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bboat_pkg/gain_servRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class gain_servRequest(genpy.Message):
  _md5sum = "be3c44e19d0c6b00b25e356c69155e2a"
  _type = "bboat_pkg/gain_servRequest"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """bool req
"""
  __slots__ = ['req']
  _slot_types = ['bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       req

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(gain_servRequest, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.req is None:
        self.req = False
    else:
      self.req = False

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self.req
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.req,) = _get_struct_B().unpack(str[start:end])
      self.req = bool(self.req)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.req
      buff.write(_get_struct_B().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 1
      (self.req,) = _get_struct_B().unpack(str[start:end])
      self.req = bool(self.req)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_B = None
def _get_struct_B():
    global _struct_B
    if _struct_B is None:
        _struct_B = struct.Struct("<B")
    return _struct_B
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bboat_pkg/gain_servResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import std_msgs.msg

class gain_servResponse(genpy.Message):
  _md5sum = "e3308e00b5029dc92caf544598a5631d"
  _type = "bboat_pkg/gain_servResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Float64 kp_1
std_msgs/Float64 ki_1
std_msgs/Float64 kd_1
std_msgs/Float64 kp_2
std_msgs/Float64 ki_2
std_msgs/Float64 kd_2


================================================================================
MSG: std_msgs/Float64
float64 data"""
  __slots__ = ['kp_1','ki_1','kd_1','kp_2','ki_2','kd_2']
  _slot_types = ['std_msgs/Float64','std_msgs/Float64','std_msgs/Float64','std_msgs/Float64','std_msgs/Float64','std_msgs/Float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       kp_1,ki_1,kd_1,kp_2,ki_2,kd_2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(gain_servResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.kp_1 is None:
        self.kp_1 = std_msgs.msg.Float64()
      if self.ki_1 is None:
        self.ki_1 = std_msgs.msg.Float64()
      if self.kd_1 is None:
        self.kd_1 = std_msgs.msg.Float64()
      if self.kp_2 is None:
        self.kp_2 = std_msgs.msg.Float64()
      if self.ki_2 is None:
        self.ki_2 = std_msgs.msg.Float64()
      if self.kd_2 is None:
        self.kd_2 = std_msgs.msg.Float64()
    else:
      self.kp_1 = std_msgs.msg.Float64()
      self.ki_1 = std_msgs.msg.Float64()
      self.kd_1 = std_msgs.msg.Float64()
      self.kp_2 = std_msgs.msg.Float64()
      self.ki_2 = std_msgs.msg.Float64()
      self.kd_2 = std_msgs.msg.Float64()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6d().pack(_x.kp_1.data, _x.ki_1.data, _x.kd_1.data, _x.kp_2.data, _x.ki_2.data, _x.kd_2.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.kp_1 is None:
        self.kp_1 = std_msgs.msg.Float64()
      if self.ki_1 is None:
        self.ki_1 = std_msgs.msg.Float64()
      if self.kd_1 is None:
        self.kd_1 = std_msgs.msg.Float64()
      if self.kp_2 is None:
        self.kp_2 = std_msgs.msg.Float64()
      if self.ki_2 is None:
        self.ki_2 = std_msgs.msg.Float64()
      if self.kd_2 is None:
        self.kd_2 = std_msgs.msg.Float64()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.kp_1.data, _x.ki_1.data, _x.kd_1.data, _x.kp_2.data, _x.ki_2.data, _x.kd_2.data,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6d().pack(_x.kp_1.data, _x.ki_1.data, _x.kd_1.data, _x.kp_2.data, _x.ki_2.data, _x.kd_2.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.kp_1 is None:
        self.kp_1 = std_msgs.msg.Float64()
      if self.ki_1 is None:
        self.ki_1 = std_msgs.msg.Float64()
      if self.kd_1 is None:
        self.kd_1 = std_msgs.msg.Float64()
      if self.kp_2 is None:
        self.kp_2 = std_msgs.msg.Float64()
      if self.ki_2 is None:
        self.ki_2 = std_msgs.msg.Float64()
      if self.kd_2 is None:
        self.kd_2 = std_msgs.msg.Float64()
      end = 0
      _x = self
      start = end
      end += 48
      (_x.kp_1.data, _x.ki_1.data, _x.kd_1.data, _x.kp_2.data, _x.ki_2.data, _x.kd_2.data,) = _get_struct_6d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6d = None
def _get_struct_6d():
    global _struct_6d
    if _struct_6d is None:
        _struct_6d = struct.Struct("<6d")
    return _struct_6d
class gain_serv(object):
  _type          = 'bboat_pkg/gain_serv'
  _md5sum = '5f632b8cb09f5ccd1ae01e67f49049ea'
  _request_class  = gain_servRequest
  _response_class = gain_servResponse
