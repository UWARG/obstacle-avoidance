"""
Test for clustering module.
"""

import pytest

from modules import lidar_detection
from modules.clustering import clustering

MAX_CLUSTER_DISTANCE = 0.5  # metres

# pylint: disable=redefined-outer-name

@pytest.fixture()
def clustering_maker() -> clustering.Clustering:  # type: ignore
    """
    Construct a clustering instance with predefined max cluster distance limit.
    """
    clustering_instance = clustering.Clustering(MAX_CLUSTER_DISTANCE)
    yield clustering_instance


@pytest.fixture()
def cluster_member_1() -> lidar_detection.LidarDetection:  # type: ignore
    """
    Creates a LidarDetection that should be clustered with cluster_member_2 and cluster_member_3.
    """
    result, detection = lidar_detection.LidarDetection.create(1.0, -7.0)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def cluster_member_2() -> lidar_detection.LidarDetection:  # type: ignore
    """
    Creates a LidarDetection that should be clustered with cluster_member_1 and cluster_member_3.
    """
    result, detection = lidar_detection.LidarDetection.create(1.0, -9.0)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def cluster_member_3() -> lidar_detection.LidarDetection:  # type: ignore
    """
    Creates a LidarDetection that should be clustered with cluster_member_1 and cluster_member_2.
    """
    result, detection = lidar_detection.LidarDetection.create(1.0, -11.0)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def cluster_outsider() -> lidar_detection.LidarDetection:  # type: ignore
    """
    Creates a LidarDetection that should be clustered on its own.
    """
    result, detection = lidar_detection.LidarDetection.create(3.0, -13.0)
    assert result
    assert detection is not None
    yield detection


class TestClustering:
    """
    Test for the Clustering.run() method.
    """

    def test_clustering(
        self,
        clustering_maker: clustering.Clustering,
        cluster_member_1: lidar_detection.LidarDetection,
        cluster_member_2: lidar_detection.LidarDetection,
        cluster_member_3: lidar_detection.LidarDetection,
        cluster_outsider: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test clustering module.
        """
        expected = None
        result, cluster = clustering_maker.run(cluster_member_1)
        assert not result
        assert cluster == expected

        expected = None
        result, cluster = clustering_maker.run(cluster_member_2)
        assert not result
        assert cluster == expected

        expected = None
        result, cluster = clustering_maker.run(cluster_member_3)
        assert not result
        assert cluster == expected

        expected = 3
        result, cluster = clustering_maker.run(cluster_outsider)
        assert result
        assert cluster is not None
        assert len(cluster.detections) == expected
