Release checklist:

* [ ] check REQUIRE files
* [ ] Run tests from previous released version against master. Ensure that there are reasonable deprecation messages, etc.
* Run tests for registered dependencies:
  * [ ] MechanismGeometries
  * [ ] MeshCatMechanisms
  * [ ] RigidBodyTreeInspector
  * [ ] MotionCaptureJointCalibration (Interact problem though)
  * [ ] RigidBodySim
  * [ ] ValkyrieRobot
  * [ ] AtlasRobot
* Run tests for unregistered dependencies:
  * [ ] LCPSim
  * [ ] HumanoidLCMSim
* [ ] Ensure that docs are up to date and that they build properly
* [ ] Update benchmark results, if relevant
* [ ] Write release notes
* [ ] Use Attobot to set up release
