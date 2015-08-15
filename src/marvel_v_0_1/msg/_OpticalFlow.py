"""autogenerated by genpy from marvel_v_0_1/OpticalFlow.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class OpticalFlow(genpy.Message):
  _md5sum = "ca586a0c3a3edbdcec70c87951b0459b"
  _type = "marvel_v_0_1/OpticalFlow"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """float32 ground_distance  # distance to ground in meters
float32 flow_x       # x-component of scaled optical flow in m
float32 flow_y       # y-component of scaled optical flow in m
uint8   quality      # quality of optical flow estimate
float32 velocity_x
float32 velocity_y


"""
  __slots__ = ['ground_distance','flow_x','flow_y','quality','velocity_x','velocity_y']
  _slot_types = ['float32','float32','float32','uint8','float32','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       ground_distance,flow_x,flow_y,quality,velocity_x,velocity_y

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(OpticalFlow, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.ground_distance is None:
        self.ground_distance = 0.
      if self.flow_x is None:
        self.flow_x = 0.
      if self.flow_y is None:
        self.flow_y = 0.
      if self.quality is None:
        self.quality = 0
      if self.velocity_x is None:
        self.velocity_x = 0.
      if self.velocity_y is None:
        self.velocity_y = 0.
    else:
      self.ground_distance = 0.
      self.flow_x = 0.
      self.flow_y = 0.
      self.quality = 0
      self.velocity_x = 0.
      self.velocity_y = 0.

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
      buff.write(_struct_3fB2f.pack(_x.ground_distance, _x.flow_x, _x.flow_y, _x.quality, _x.velocity_x, _x.velocity_y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 21
      (_x.ground_distance, _x.flow_x, _x.flow_y, _x.quality, _x.velocity_x, _x.velocity_y,) = _struct_3fB2f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3fB2f.pack(_x.ground_distance, _x.flow_x, _x.flow_y, _x.quality, _x.velocity_x, _x.velocity_y))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 21
      (_x.ground_distance, _x.flow_x, _x.flow_y, _x.quality, _x.velocity_x, _x.velocity_y,) = _struct_3fB2f.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3fB2f = struct.Struct("<3fB2f")