
"use strict";

let SetDrawingTrajectory = require('./SetDrawingTrajectory.js')
let GetKinematicsPose = require('./GetKinematicsPose.js')
let SetActuatorState = require('./SetActuatorState.js')
let SetJointPosition = require('./SetJointPosition.js')
let GetJointPosition = require('./GetJointPosition.js')
let SetKinematicsPose = require('./SetKinematicsPose.js')

module.exports = {
  SetDrawingTrajectory: SetDrawingTrajectory,
  GetKinematicsPose: GetKinematicsPose,
  SetActuatorState: SetActuatorState,
  SetJointPosition: SetJointPosition,
  GetJointPosition: GetJointPosition,
  SetKinematicsPose: SetKinematicsPose,
};
