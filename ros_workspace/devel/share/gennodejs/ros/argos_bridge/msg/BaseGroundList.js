// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BaseGround = require('./BaseGround.js');

//-----------------------------------------------------------

class BaseGroundList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n = null;
      this.baseGrounds = null;
    }
    else {
      if (initObj.hasOwnProperty('n')) {
        this.n = initObj.n
      }
      else {
        this.n = 0;
      }
      if (initObj.hasOwnProperty('baseGrounds')) {
        this.baseGrounds = initObj.baseGrounds
      }
      else {
        this.baseGrounds = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BaseGroundList
    // Serialize message field [n]
    bufferOffset = _serializer.uint32(obj.n, buffer, bufferOffset);
    // Serialize message field [baseGrounds]
    // Serialize the length for message field [baseGrounds]
    bufferOffset = _serializer.uint32(obj.baseGrounds.length, buffer, bufferOffset);
    obj.baseGrounds.forEach((val) => {
      bufferOffset = BaseGround.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BaseGroundList
    let len;
    let data = new BaseGroundList(null);
    // Deserialize message field [n]
    data.n = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [baseGrounds]
    // Deserialize array length for message field [baseGrounds]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.baseGrounds = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.baseGrounds[i] = BaseGround.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.baseGrounds.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/BaseGroundList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69c1c089ae74b076579a8bd75e220110';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 n
    BaseGround[] baseGrounds
    
    ================================================================================
    MSG: argos_bridge/BaseGround
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
    const resolved = new BaseGroundList(null);
    if (msg.n !== undefined) {
      resolved.n = msg.n;
    }
    else {
      resolved.n = 0
    }

    if (msg.baseGrounds !== undefined) {
      resolved.baseGrounds = new Array(msg.baseGrounds.length);
      for (let i = 0; i < resolved.baseGrounds.length; ++i) {
        resolved.baseGrounds[i] = BaseGround.Resolve(msg.baseGrounds[i]);
      }
    }
    else {
      resolved.baseGrounds = []
    }

    return resolved;
    }
};

module.exports = BaseGroundList;
