Recommended way to run teh BT tick analysis tool:

```python
docker logs -f mk4_ws-robot_bt-1 2>&1 | ./bt_tick_stats.py --every 5 --top 10
```
