# rviz_topic_button_panel

Reusable RViz panel with two buttons that publish `std_msgs/msg/Empty`.
Configure the labels and topics in the RViz display config:

```yaml
Panels:
  - Class: rviz_topic_button_panel/EmptyTopicButtonPanel
    Name: Motion Planning
    Description: Drag the target marker, then plan and execute
    Button 1 Label: Plan
    Button 1 Topic: /motion_planning/plan
    Button 2 Label: Execute
    Button 2 Topic: /motion_planning/execute
```
