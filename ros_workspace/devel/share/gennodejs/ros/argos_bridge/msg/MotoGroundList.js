// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MotoGround = require('./MotoGround.js');

//-----------------------------------------------------------

class MotoGroundList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n = null;
      this.motoGrounds = null;
    }
    else {
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('motoGrounds')) {
        this.motoGrounds = initObj.motoGrounds
      }
      else {
        this.motoGrounds = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotoGroundList
    // Serialize message field [n]
    bufferOffset = _serializer.uint32(obj.n, buffer, bufferOffset);
    // Serialize message field [motoGrounds]
    // Serialize the length for message field [motoGrounds]
    bufferOffset = _serializer.uint32(obj.motoGrounds.length, buffer, bufferOffset);
    obj.motoGrounds.forEach((val) => {
      bufferOffset = MotoGround.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotoGroundList
    let len;
    let data = new MotoGroundList(null);
    // Deserialize message field [n]
    data.n = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [motoGrounds]
    // Deserialize array length for message field [motoGrounds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.motoGrounds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.motoGrounds[i] = MotoGround.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.motoGrounds.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/MotoGroundList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '49d994c0a5d5dc1356fd2c48d2b519a1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 n
    MotoGround[] motoGrounds
    
    ================================================================================
    MSG: argos_bridge/MotoGround
    float32 value
    float32 offset_x
    float32 offset_y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotoGroundList(null);
    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.motoGrounds !== undefined) {
      resolved.motoGrounds = new Array(msg.motoGrounds.length);
      for (let i = 0; i < resolved.motoGrounds.length; ++i) {
        resolved.motoGrounds[i] = MotoGround.Resolve(msg.motoGrounds[i]);
      }
    }
    else {
      resolved.motoGrounds = []
    }

    return resolved;
    }
};

module.exports = MotoGroundList;
