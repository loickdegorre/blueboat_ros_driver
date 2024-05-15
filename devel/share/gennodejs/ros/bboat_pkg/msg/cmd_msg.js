// Auto-generated. Do not edit!

// (in-package bboat_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class cmd_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.u1 = null;
      this.u2 = null;
    }
    else {
      if (initObj.hasOwnProperty('u1')) {
        this.u1 = initObj.u1
      }
      else {
        this.u1 = new std_msgs.msg.Float64();
      }
      if (initObj.hasOwnProperty('u2')) {
        this.u2 = initObj.u2
      }
      else {
        this.u2 = new std_msgs.msg.Float64();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_msg
    // Serialize message field [u1]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.u1, buffer, bufferOffset);
    // Serialize message field [u2]
    bufferOffset = std_msgs.msg.Float64.serialize(obj.u2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_msg
    let len;
    let data = new cmd_msg(null);
    // Deserialize message field [u1]
    data.u1 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    // Deserialize message field [u2]
    data.u2 = std_msgs.msg.Float64.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bboat_pkg/cmd_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '565f2132c1adf413c58d2ae5bf8d97b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Float64 u1
    std_msgs/Float64 u2
    
    ================================================================================
    MSG: std_msgs/Float64
    float64 data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_msg(null);
    if (msg.u1 !== undefined) {
      resolved.u1 = std_msgs.msg.Float64.Resolve(msg.u1)
    }
    else {
      resolved.u1 = new std_msgs.msg.Float64()
    }

    if (msg.u2 !== undefined) {
      resolved.u2 = std_msgs.msg.Float64.Resolve(msg.u2)
    }
    else {
      resolved.u2 = new std_msgs.msg.Float64()
    }

    return resolved;
    }
};

module.exports = cmd_msg;
