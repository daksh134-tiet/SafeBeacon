# SafeBeacon
SafeBeacon is an embedded hardware project that turns any vehicle into a self-reporting emergency beacon. When a crash is detected, the device autonomously transmits GPS coordinates, pre-crash velocity data, and an SOS alert — no phone, no bystander, no app required.
This project is a proof-of-concept for hardware-layer emergency infrastructure, designed to complement software-based vehicle safety platforms operating in India.

What It Does
ModeTriggerActionCrash DetectionIMU G-force > 3.5G for >80msTransmits SOS packet: GPS coords + pre-crash velocity trace + severity estimateDriver AlertManual SOS buttonImmediate emergency dispatch with live locationVehicle HealthEvery 10 seconds (ignition on)Streams OBD2 telemetry: RPM, speed, coolant temp, battery voltage, fault codesTrip AnalyticsPer tripDistance, harsh braking events, overspeed flags, idle timeFalse Alarm PreventionPost-trigger, 15 secondsDriver cancel window before SOS is transmitted
