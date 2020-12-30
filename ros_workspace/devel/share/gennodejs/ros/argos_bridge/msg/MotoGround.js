// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotoGround {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.value = null;
      this.offset_x = null;
      this.offset_y = null;
    }
    else {
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
      if (initObj.hasOwnProperty('offset_x')) {
        this.offset_x = initObj.offset_x
      }
      else {
        this.offset_x = 0.0;
      }
      if (initObj.hasOwnProperty('offset_y')) {
        this.offset_y = initObj.offset_y
      }
      else {
        this.offset_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotoGround
    // Serialize message field [value]
    bufferOffset = _serializer.float32(obj.value, buffer, bufferOffset);
    // Serialize message field [offset_x]
    bufferOffset = _serializer.float32(obj.offset_x, buffer, bufferOffset);
    // Serialize message field [offset_y]
    bufferOffset = _serializer.float32(obj.offset_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotoGround
    let len;
    let data = new MotoGround(null);
    // Deserialize message field [value]
    data.value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [offset_x]
    data.offset_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [offset_y]
    data.offset_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/MotoGround';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df359e3890070432de3f37977284cdda';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new MotoGround(null);
    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    if (msg.offset_x !== undefined) {
      resolved.offset_x = msg.offset_x;
    }
    else {
      resolved.offset_x = 0.0
    }

    if (msg.offset_y !== undefined) {
      resolved.offset_y = msg.offset_y;
    }
    else {
      resolved.offset_y = 0.0
    }

    return resolved;
    }
};

module.exports = MotoGround;
