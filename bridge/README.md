# bridge — uniform input node

`CustomMsg → PointCloud2(ring, time)` bridge so baselines that require
`sensor_msgs/PointCloud2` consume an **identical** input stream (plan §2).

**Not needed yet.** The first two baselines (FAST-LIO, faster-lio) ingest Livox
`CustomMsg` natively, so they read the raw bag topic directly. This node is
implemented when the first `PointCloud2`-only baseline (e.g. DLIO) is admitted.
