// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DistScan = require('./DistScan.js');

//-----------------------------------------------------------

class DistScanList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n = null;
      this.scan = null;
    }
    else {
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('scan')) {
        this.scan = initObj.scan
      }
      else {
        this.scan = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DistScanList
    // Serialize message field [n]
    bufferOffset = _serializer.int32(obj.n, buffer, bufferOffset);
    // Serialize message field [scan]
    // Serialize the length for message field [scan]
    bufferOffset = _serializer.uint32(obj.scan.length, buffer, bufferOffset);
    obj.scan.forEach((val) => {
      bufferOffset = DistScan.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DistScanList
    let len;
    let data = new DistScanList(null);
    // Deserialize message field [n]
    data.n = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [scan]
    // Deserialize array length for message field [scan]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.scan = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.scan[i] = DistScan.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.scan.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/DistScanList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7fb89b86c4713c9df9a68e4b0fd602da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 n
    DistScan[] scan
    ================================================================================
    MSG: argos_bridge/DistScan
    float32 range
    float32 angle
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DistScanList(null);
    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.scan !== undefined) {
      resolved.scan = new Array(msg.scan.length);
      for (let i = 0; i < resolved.scan.length; ++i) {
        resolved.scan[i] = DistScan.Resolve(msg.scan[i]);
      }
    }
    else {
      resolved.scan = []
    }

    return resolved;
    }
};

module.exports = DistScanList;
