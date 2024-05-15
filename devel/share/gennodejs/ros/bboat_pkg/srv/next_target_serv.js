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

class next_target_servRequest {
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
    // Serializes a message object of type next_target_servRequest
    // Serialize message field [req]
    bufferOffset = _serializer.bool(obj.req, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type next_target_servRequest
    let len;
    let data = new next_target_servRequest(null);
    // Deserialize message field [req]
    data.req = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/next_target_servRequest';
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
    const resolved = new next_target_servRequest(null);
    if (msg.req !== undefined) {
      resolved.req = msg.req;
    }
    else {
      resolved.req = false
    }

    return resolved;
    }
};

class next_target_servResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.next_trgt_pose = null;
      this.continuing_mission = null;
    }
    else {
      if (initObj.hasOwnProperty('next_trgt_pose')) {
        this.next_trgt_pose = initObj.next_trgt_pose
      }
      else {
        this.next_trgt_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('continuing_mission')) {
        this.continuing_mission = initObj.continuing_mission
      }
      else {
        this.continuing_mission = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type next_target_servResponse
    // Serialize message field [next_trgt_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.next_trgt_pose, buffer, bufferOffset);
    // Serialize message field [continuing_mission]
    bufferOffset = _serializer.bool(obj.continuing_mission, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type next_target_servResponse
    let len;
    let data = new next_target_servResponse(null);
    // Deserialize message field [next_trgt_pose]
    data.next_trgt_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [continuing_mission]
    data.continuing_mission = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 57;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bboat_pkg/next_target_servResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '134a9f41a53729bf70f386ba88fcc491';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose next_trgt_pose
    bool continuing_mission
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new next_target_servResponse(null);
    if (msg.next_trgt_pose !== undefined) {
      resolved.next_trgt_pose = geometry_msgs.msg.Pose.Resolve(msg.next_trgt_pose)
    }
    else {
      resolved.next_trgt_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.continuing_mission !== undefined) {
      resolved.continuing_mission = msg.continuing_mission;
    }
    else {
      resolved.continuing_mission = false
    }

    return resolved;
    }
};

module.exports = {
  Request: next_target_servRequest,
  Response: next_target_servResponse,
  md5sum() { return 'd9ce4b52e25eaaef6e3a53a636b0b720'; },
  datatype() { return 'bboat_pkg/next_target_serv'; }
};
