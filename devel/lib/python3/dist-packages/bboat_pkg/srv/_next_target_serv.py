# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from bboat_pkg/next_target_servRequest.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class next_target_servRequest(genpy.Message):
  _md5sum = "be3c44e19d0c6b00b25e356c69155e2a"
  _type = "bboat_pkg/next_target_servRequest"
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
      super(next_target_servRequest, self).__init__(*args, **kwds)
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
"""autogenerated by genpy from bboat_pkg/next_target_servResponse.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class next_target_servResponse(genpy.Message):
  _md5sum = "134a9f41a53729bf70f386ba88fcc491"
  _type = "bboat_pkg/next_target_servResponse"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """geometry_msgs/Pose next_trgt_pose
bool continuing_mission

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['next_trgt_pose','continuing_mission']
  _slot_types = ['geometry_msgs/Pose','bool']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       next_trgt_pose,continuing_mission

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(next_target_servResponse, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.next_trgt_pose is None:
        self.next_trgt_pose = geometry_msgs.msg.Pose()
      if self.continuing_mission is None:
        self.continuing_mission = False
    else:
      self.next_trgt_pose = geometry_msgs.msg.Pose()
      self.continuing_mission = False

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
      buff.write(_get_struct_7dB().pack(_x.next_trgt_pose.position.x, _x.next_trgt_pose.position.y, _x.next_trgt_pose.position.z, _x.next_trgt_pose.orientation.x, _x.next_trgt_pose.orientation.y, _x.next_trgt_pose.orientation.z, _x.next_trgt_pose.orientation.w, _x.continuing_mission))
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
      if self.next_trgt_pose is None:
        self.next_trgt_pose = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 57
      (_x.next_trgt_pose.position.x, _x.next_trgt_pose.position.y, _x.next_trgt_pose.position.z, _x.next_trgt_pose.orientation.x, _x.next_trgt_pose.orientation.y, _x.next_trgt_pose.orientation.z, _x.next_trgt_pose.orientation.w, _x.continuing_mission,) = _get_struct_7dB().unpack(str[start:end])
      self.continuing_mission = bool(self.continuing_mission)
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
      buff.write(_get_struct_7dB().pack(_x.next_trgt_pose.position.x, _x.next_trgt_pose.position.y, _x.next_trgt_pose.position.z, _x.next_trgt_pose.orientation.x, _x.next_trgt_pose.orientation.y, _x.next_trgt_pose.orientation.z, _x.next_trgt_pose.orientation.w, _x.continuing_mission))
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
      if self.next_trgt_pose is None:
        self.next_trgt_pose = geometry_msgs.msg.Pose()
      end = 0
      _x = self
      start = end
      end += 57
      (_x.next_trgt_pose.position.x, _x.next_trgt_pose.position.y, _x.next_trgt_pose.position.z, _x.next_trgt_pose.orientation.x, _x.next_trgt_pose.orientation.y, _x.next_trgt_pose.orientation.z, _x.next_trgt_pose.orientation.w, _x.continuing_mission,) = _get_struct_7dB().unpack(str[start:end])
      self.continuing_mission = bool(self.continuing_mission)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_7dB = None
def _get_struct_7dB():
    global _struct_7dB
    if _struct_7dB is None:
        _struct_7dB = struct.Struct("<7dB")
    return _struct_7dB
class next_target_serv(object):
  _type          = 'bboat_pkg/next_target_serv'
  _md5sum = 'd9ce4b52e25eaaef6e3a53a636b0b720'
  _request_class  = next_target_servRequest
  _response_class = next_target_servResponse
