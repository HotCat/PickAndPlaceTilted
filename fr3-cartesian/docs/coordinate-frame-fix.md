# FR3 workpiece-reference diagnosis — 2026-11-18

The app previously treated the controller's flange feedback as robot-base feedback. On the tested FR3 (V3.8.25-QX, internal v3.8.5.1, SDK v2.1.5), read-only RPC measurements show that both flange and TCP feedback are expressed in the active Wobj. Teaching with a nonzero workpiece frame therefore stored local points as though they were base points. Uploading that plane produced an incorrect controller frame and unexpected WebUI coordinates and jogging axes.

The corrected calculation composes the entire pose, including orientation:

```text
T_base_flange = T_base_wobj × T_wobj_flange
T_base_tcp    = T_base_flange × T_flange_tcp
```

It reads the active tool number, Wobj number and both offsets before and after the live flange query. If the references change, it retries only these reads once, then rejects an unsettled reference. It does not combine an older SDK state-packet pose with a newer workpiece offset. Joint angles and controller stop/fault flags still come from the monitored live state stream.

## Evidence and limitations

Local read-only observations compared active-Wobj flange/TCP poses with a calculation-only controller workpiece solver. Composing the workpiece transform explained the large apparent origin displacement and agreed with the solver. These observations identified a frame-convention bug; they did not establish physical positioning accuracy or prove every old point used the same frame. Original cell measurements are not distributed in this repository.

`tests/test_sdk_wire.py` now exercises the same convention with synthetic XML-RPC responses: a workpiece at `[100, 200, 300]` mm rotated +90° about base Z, a local flange at `[20, 30, 40]` mm with Rx=180°, and a 100 mm tool Z offset must produce base TCP `[70, 220, 240]` mm. The expected position is independently calculable, and the test verifies orientation composition as well as translation. No SDK constructor, network sockets or robot commands are used.

Offline transformation and serialization tests cannot establish physical axis directions, calibration accuracy or stopping behavior on the installed cell. These remain supervised commissioning checks.

## Stored-data handling

Schema 3 records `robot_base` or `legacy_workpiece_unknown` reference metadata. Old real-robot plane points and contacts without provenance remain available but cannot be mixed with fresh captures. Unknown-reference planes cannot be uploaded, orientation-locked or used for motion. The existing TCP offset can still be verified and used; a fixed Wobj reference throughout the original pivot measurement does not change the fitted flange-to-TCP offset.

The repair workflow in the [English](../README.md#repairing-planes-captured-by-older-versions) and [简体中文](../README.zh-CN.md) guides requires fresh O/X/Y teaching. The incorrect Wobj1 already on the controller remains until the operator explicitly replaces it. Near the freshly taught origin, XYZ should be near zero; tool RX/RY/RZ relative to the plane need not be zero.

For the app's own XYZ movement, the controller must still use identity Wobj0: the app transforms its local plane targets to base and sends `MoveL(user=0)`. For official WebUI Wobj jogging, explicitly upload the reviewed new plane and select that Wobj slot.
