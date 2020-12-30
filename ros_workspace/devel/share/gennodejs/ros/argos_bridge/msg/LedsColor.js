// Auto-generated. Do not edit!

// (in-package argos_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LedsColor {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.color = null;
      this.led_number = null;
      this.intensity = null;
    }
    else {
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = new std_msgs.msg.ColorRGBA();
      }
      if (initObj.hasOwnProperty('led_number')) {
        this.led_number = initObj.led_number
      }
      else {
        this.led_number = 0;
      }
      if (initObj.hasOwnProperty('intensity')) {
        this.intensity = initObj.intensity
      }
      else {
        this.intensity = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LedsColor
    // Serialize message field [color]
    bufferOffset = std_msgs.msg.ColorRGBA.serialize(obj.color, buffer, bufferOffset);
    // Serialize message field [led_number]
    bufferOffset = _serializer.uint32(obj.led_number, buffer, bufferOffset);
    // Serialize message field [intensity]
    bufferOffset = _serializer.uint8(obj.intensity, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LedsColor
    let len;
    let data = new LedsColor(null);
    // Deserialize message field [color]
    data.color = std_msgs.msg.ColorRGBA.deserialize(buffer, bufferOffset);
    // Deserialize message field [led_number]
    data.led_number = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [intensity]
    data.intensity = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'argos_bridge/LedsColor';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '62998bbed77dd665705d07b81ccc0148';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/ColorRGBA color
    uint32 led_number
    uint8 intensity
    
    ================================================================================
    MSG: std_msgs/ColorRGBA
    float32 r
    float32 g
    float32 b
    float32 a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LedsColor(null);
    if (msg.color !== undefined) {
      resolved.color = std_msgs.msg.ColorRGBA.Resolve(msg.color)
    }
    else {
      resolved.color = new std_msgs.msg.ColorRGBA()
    }

    if (msg.led_number !== undefined) {
      resolved.led_number = msg.led_number;
    }
    else {
      resolved.led_number = 0
    }

    if (msg.intensity !== undefined) {
      resolved.intensity = msg.intensity;
    }
    else {
      resolved.intensity = 0
    }

    return resolved;
    }
};

module.exports = LedsColor;
