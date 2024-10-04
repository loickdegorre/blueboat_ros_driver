// Auto-generated. Do not edit!

// (in-package bboat_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class mode_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.mission = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = '';
      }
      if (initObj.hasOwnProperty('mission')) {
        this.mission = initObj.mission
      }
      else {
        this.mission = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mode_msg
    // Serialize message field [mode]
    bufferOffset = _serializer.string(obj.mode, buffer, bufferOffset);
    // Serialize message field [mission]
    bufferOffset = _serializer.string(obj.mission, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mode_msg
    let len;
    let data = new mode_msg(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [mission]
    data.mission = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.mode);
    length += _getByteLength(object.mission);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bboat_pkg/mode_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6609a085032a3aa05e4e4147b4c8d4d3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string mode
    string mission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mode_msg(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = ''
    }

    if (msg.mission !== undefined) {
      resolved.mission = msg.mission;
    }
    else {
      resolved.mission = ''
    }

    return resolved;
    }
};

module.exports = mode_msg;
