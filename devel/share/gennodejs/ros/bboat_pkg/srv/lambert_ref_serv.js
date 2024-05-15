// Auto-generated. Do not edit!

// (in-package bboat_pkg.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class lambert_ref_servRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.req = null;
    }
    else {
      if (initObj.hasOwnProperty('req')) {
        this.req = initObj.req
      }
      else {
        this.req = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lambert_ref_servRequest
    // Serialize message field [req]
    bufferOffset = _serializer.bool(obj.req, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lambert_ref_servRequest
    let len;
    let data = new lambert_ref_servRequest(null);
    // Deserialize message field [req]
    data.req = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/lambert_ref_servRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be3c44e19d0c6b00b25e356c69155e2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool req
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lambert_ref_servRequest(null);
    if (msg.req !== undefined) {
      resolved.req = msg.req;
    }
    else {
      resolved.req = false
    }

    return resolved;
    }
};

class lambert_ref_servResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lambert_ref = null;
    }
    else {
      if (initObj.hasOwnProperty('lambert_ref')) {
        this.lambert_ref = initObj.lambert_ref
      }
      else {
        this.lambert_ref = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lambert_ref_servResponse
    // Serialize message field [lambert_ref]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.lambert_ref, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lambert_ref_servResponse
    let len;
    let data = new lambert_ref_servResponse(null);
    // Deserialize message field [lambert_ref]
    data.lambert_ref = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/lambert_ref_servResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd9a86cd1b06af5288af973c72e093b29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point lambert_ref
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lambert_ref_servResponse(null);
    if (msg.lambert_ref !== undefined) {
      resolved.lambert_ref = geometry_msgs.msg.Point.Resolve(msg.lambert_ref)
    }
    else {
      resolved.lambert_ref = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = {
  Request: lambert_ref_servRequest,
  Response: lambert_ref_servResponse,
  md5sum() { return '2f9744e749801011d7a2f5581e48ef29'; },
  datatype() { return 'bboat_pkg/lambert_ref_serv'; }
};
