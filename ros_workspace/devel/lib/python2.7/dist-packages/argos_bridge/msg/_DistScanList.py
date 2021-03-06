# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from argos_bridge/DistScanList.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import argos_bridge.msg

class DistScanList(genpy.Message):
  _md5sum = "7fb89b86c4713c9df9a68e4b0fd602da"
  _type = "argos_bridge/DistScanList"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """int32 n
DistScan[] scan
================================================================================
MSG: argos_bridge/DistScan
float32 range
float32 angle"""
  __slots__ = ['n','scan']
  _slot_types = ['int32','argos_bridge/DistScan[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       n,scan

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(DistScanList, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.n is None:
        self.n = 0
      if self.scan is None:
        self.scan = []
    else:
      self.n = 0
      self.scan = []

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
      _x = self.n
      buff.write(_get_struct_i().pack(_x))
      length = len(self.scan)
      buff.write(_struct_I.pack(length))
      for val1 in self.scan:
        _x = val1
        buff.write(_get_struct_2f().pack(_x.range, _x.angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.scan is None:
        self.scan = None
      end = 0
      start = end
      end += 4
      (self.n,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.scan = []
      for i in range(0, length):
        val1 = argos_bridge.msg.DistScan()
        _x = val1
        start = end
        end += 8
        (_x.range, _x.angle,) = _get_struct_2f().unpack(str[start:end])
        self.scan.append(val1)
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
      _x = self.n
      buff.write(_get_struct_i().pack(_x))
      length = len(self.scan)
      buff.write(_struct_I.pack(length))
      for val1 in self.scan:
        _x = val1
        buff.write(_get_struct_2f().pack(_x.range, _x.angle))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.scan is None:
        self.scan = None
      end = 0
      start = end
      end += 4
      (self.n,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.scan = []
      for i in range(0, length):
        val1 = argos_bridge.msg.DistScan()
        _x = val1
        start = end
        end += 8
        (_x.range, _x.angle,) = _get_struct_2f().unpack(str[start:end])
        self.scan.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2f = None
def _get_struct_2f():
    global _struct_2f
    if _struct_2f is None:
        _struct_2f = struct.Struct("<2f")
    return _struct_2f
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
