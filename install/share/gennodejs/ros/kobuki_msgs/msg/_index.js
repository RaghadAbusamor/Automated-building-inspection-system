
"use strict";

let ControllerInfo = require('./ControllerInfo.js');
let MotorPower = require('./MotorPower.js');
let Led = require('./Led.js');
let Sound = require('./Sound.js');
let DigitalOutput = require('./DigitalOutput.js');
let KeyboardInput = require('./KeyboardInput.js');
let DockInfraRed = require('./DockInfraRed.js');
let SensorState = require('./SensorState.js');
let WheelDropEvent = require('./WheelDropEvent.js');
let ExternalPower = require('./ExternalPower.js');
let RobotStateEvent = require('./RobotStateEvent.js');
let BumperEvent = require('./BumperEvent.js');
let PowerSystemEvent = require('./PowerSystemEvent.js');
let ButtonEvent = require('./ButtonEvent.js');
let ScanAngle = require('./ScanAngle.js');
let CliffEvent = require('./CliffEvent.js');
let DigitalInputEvent = require('./DigitalInputEvent.js');
let VersionInfo = require('./VersionInfo.js');
let AutoDockingAction = require('./AutoDockingAction.js');
let AutoDockingFeedback = require('./AutoDockingFeedback.js');
let AutoDockingActionGoal = require('./AutoDockingActionGoal.js');
let AutoDockingActionFeedback = require('./AutoDockingActionFeedback.js');
let AutoDockingGoal = require('./AutoDockingGoal.js');
let AutoDockingResult = require('./AutoDockingResult.js');
let AutoDockingActionResult = require('./AutoDockingActionResult.js');

module.exports = {
  ControllerInfo: ControllerInfo,
  MotorPower: MotorPower,
  Led: Led,
  Sound: Sound,
  DigitalOutput: DigitalOutput,
  KeyboardInput: KeyboardInput,
  DockInfraRed: DockInfraRed,
  SensorState: SensorState,
  WheelDropEvent: WheelDropEvent,
  ExternalPower: ExternalPower,
  RobotStateEvent: RobotStateEvent,
  BumperEvent: BumperEvent,
  PowerSystemEvent: PowerSystemEvent,
  ButtonEvent: ButtonEvent,
  ScanAngle: ScanAngle,
  CliffEvent: CliffEvent,
  DigitalInputEvent: DigitalInputEvent,
  VersionInfo: VersionInfo,
  AutoDockingAction: AutoDockingAction,
  AutoDockingFeedback: AutoDockingFeedback,
  AutoDockingActionGoal: AutoDockingActionGoal,
  AutoDockingActionFeedback: AutoDockingActionFeedback,
  AutoDockingGoal: AutoDockingGoal,
  AutoDockingResult: AutoDockingResult,
  AutoDockingActionResult: AutoDockingActionResult,
};
