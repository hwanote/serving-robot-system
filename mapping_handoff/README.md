# Robot 01 Mapping Handoff

Team integration sources for:
Start Mapping -> automatic exploration -> Stop Mapping
-> map review -> Save Map.

ROS source: ros_ws/src/serving_robot_mapping
Backend: integration/mapping_api.py, integration/ingest_map.py
Review: review/backend/map_review.py
Frontend: review/frontend/MapReviewPanel.tsx

The ROS package must be installed into the team's ROS workspace.
The frontend components require the team's existing React
dependencies and integration with its MappingPanel.

FastAPI example: import mapping_api.router and include it in the
team app, replacing the current 501 mapping endpoints.

The standalone FastAPI app is for integration testing. It is not
a production process manager.

Remaining before operational use:
- Integrate review strokes with the existing Save Map button.
- Validate ROS start/stop lifecycle and child-process cancellation.
- Test actual Robot 01 mapping in Gazebo.
- Ensure final map is supplied to the navigation team.

No Robot 02, serving goals, stop-area calculation or
automatic authorization of a Nav2 runtime map.
