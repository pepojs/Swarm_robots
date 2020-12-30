// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Vector3 = require('./Vector3.js');

//-----------------------------------------------------------

class Position {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.orientation = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new Vector3();
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = new Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Position
    // Serialize message field [position]
    bufferOffset = Vector3.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = Vector3.serialize(obj.orientation, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Position
    let len;
    let data = new Position(null);
    // Deserialize message field [position]
    data.position = Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/Position';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '884731a1adb9599d204c6712c1265f2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Vector3 position
    Vector3 orientation
    
    ================================================================================
    MSG: argos_bridge/Vector3
    float32 x
    float32 y
    float32 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Position(null);
    if (msg.position !== undefined) {
      resolved.position = Vector3.Resolve(msg.position)
    }
    else {
      resolved.position = new Vector3()
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = Vector3.Resolve(msg.orientation)
    }
    else {
      resolved.orientation = new Vector3()
    }

    return resolved;
    }
};

module.exports = Position;
