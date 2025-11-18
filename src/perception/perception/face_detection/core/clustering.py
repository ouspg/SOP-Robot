"""
Links online clustering algorithm for face tracking.

Original: https://github.com/QEDan/links_clustering
Reference: https://arxiv.org/abs/1801.10123
Modified for SOP-Robot project.
"""

import logging
import time
import uuid
from typing import List, Dict, Optional

import numpy as np
from scipy.spatial.distance import cosine

CONVERSATION_THRESHOLD = 15  # seconds


class Subcluster:
    """Represents a subcluster and tracks connected subclusters over time."""

    def __init__(self, initial_vector: np.ndarray, store_vectors: bool = False, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        self.vectors: List[np.ndarray] = [initial_vector] if store_vectors else []
        self.centroid: np.ndarray = initial_vector
        self.vector_count: int = 1
        self.store_vectors = store_vectors
        self.connected_subclusters: set["Subcluster"] = set()

        # Conversation tracking
        now = time.time()
        self.last_seen: float = now
        self.conversations: List[Dict] = [{"start_time": now, "end_time": now, "duration": 0.0}]
        self.total_time_on_camera: float = 0.0

    def add(self, vector: np.ndarray):
        """Add a new vector to the subcluster and update centroid and conversation info."""
        if self.store_vectors:
            self.vectors.append(vector)

        self.centroid = (self.centroid * self.vector_count + vector) / (self.vector_count + 1)
        self.vector_count += 1

        now = time.time()
        if now - self.last_seen <= CONVERSATION_THRESHOLD:
            # Update ongoing conversation
            conv = self.conversations[-1]
            conv["end_time"] = now
            conv["duration"] = conv["end_time"] - conv["start_time"]
            self.total_time_on_camera += now - self.last_seen
        else:
            # Start new conversation
            self.conversations.append({"start_time": now, "end_time": now, "duration": 0.0})

        self.last_seen = now


class Cluster:
    """Represents a cluster containing multiple subclusters."""

    def __init__(self, subcluster: Subcluster, logger: Optional[logging.Logger] = None):
        self.logger = logger or logging.getLogger(__name__)
        self.subclusters: List[Subcluster] = [subcluster]
        self.id: str = str(uuid.uuid4())

    @classmethod
    def from_dict(cls, data: Dict, logger: Optional[logging.Logger] = None) -> "Cluster":
        cluster = cls(subcluster=None, logger=logger)
        cluster.id = data["id"]
        cluster.subclusters = [Subcluster(**sc_data) for sc_data in data["subclusters"]]
        return cluster

    def add_subcluster(self, subcluster: Subcluster):
        self.subclusters.append(subcluster)

    def merge_subclusters(self, sc_idx1: int, sc_idx2: int, delete_merged: bool = True):
        """Merge subcluster sc_idx2 into sc_idx1."""
        sc1 = self.subclusters[sc_idx1]
        sc2 = self.subclusters[sc_idx2]

        if sc1.store_vectors:
            sc1.vectors.extend(sc2.vectors)

        # Update centroid and vector count
        sc1.centroid = (sc1.centroid * sc1.vector_count + sc2.centroid * sc2.vector_count) / (sc1.vector_count + sc2.vector_count)
        sc1.vector_count += sc2.vector_count

        # Merge connected subclusters
        sc1.connected_subclusters.discard(sc2)
        for sc in sc2.connected_subclusters:
            sc.connected_subclusters.discard(sc2)
            if sc != sc1:
                sc.connected_subclusters.add(sc1)
        sc1.connected_subclusters.update(sc2.connected_subclusters)

        # Merge conversations
        sc1.conversations = Cluster._merge_conversations([sc1.conversations, sc2.conversations])

        if delete_merged:
            self.subclusters.pop(sc_idx2)
            for sc in self.subclusters:
                sc.connected_subclusters.discard(sc2)

    def calculate_conversation_list(self) -> List[Dict]:
        """Return combined conversations from all subclusters."""
        return Cluster._merge_conversations([sc.conversations for sc in self.subclusters])

    @staticmethod
    def _merge_conversations(conversation_lists: List[List[Dict]]) -> List[Dict]:
        merged = []
        all_convs = [conv for lst in conversation_lists for conv in lst]
        all_convs = sorted(all_convs, key=lambda x: x["start_time"])

        for conv in all_convs:
            if not merged or conv["start_time"] >= merged[-1]["end_time"] + CONVERSATION_THRESHOLD:
                merged.append(conv)
            else:
                merged[-1]["end_time"] = max(merged[-1]["end_time"], conv["end_time"])
                merged[-1]["duration"] = merged[-1]["end_time"] - merged[-1]["start_time"]
        return merged

    def as_dict(self) -> Dict:
        return {"id": self.id, "conversations": self.calculate_conversation_list()}


class LinksCluster:
    """Online clustering for streaming face embeddings."""

    def __init__(
        self,
        cluster_similarity_threshold: float,
        subcluster_similarity_threshold: float,
        pair_similarity_maximum: float,
        store_vectors: bool = False,
        logger: Optional[logging.Logger] = None,
    ):
        self.logger = logger or logging.getLogger(__name__)
        self.clusters: List[Cluster] = []
        self.cluster_similarity_threshold = cluster_similarity_threshold
        self.subcluster_similarity_threshold = subcluster_similarity_threshold
        self.pair_similarity_maximum = pair_similarity_maximum
        self.store_vectors = store_vectors

    def predict(self, new_vector: np.ndarray) -> Dict:
        """Assign a new vector to a cluster/subcluster or create a new one."""
        if not self.clusters:
            cluster = Cluster(Subcluster(new_vector, store_vectors=self.store_vectors), logger=self.logger)
            self.clusters.append(cluster)
            return cluster.as_dict()

        best_sc, best_similarity, best_cluster_idx, best_sc_idx = None, -np.inf, None, None

        # Find best matching subcluster
        for cl_idx, cluster in enumerate(self.clusters):
            for sc_idx, sc in enumerate(cluster.subclusters):
                sim = 1.0 - cosine(new_vector, sc.centroid)
                if sim > best_similarity:
                    best_sc, best_similarity = sc, sim
                    best_cluster_idx, best_sc_idx = cl_idx, sc_idx

        if best_similarity >= self.subcluster_similarity_threshold:
            best_sc.add(new_vector)
            self.update_cluster(best_cluster_idx, best_sc_idx)
            self.logger.debug("Vector added to existing subcluster.")
            return self.clusters[best_cluster_idx].as_dict()
        else:
            # Create new subcluster
            new_sc = Subcluster(new_vector, store_vectors=self.store_vectors)
            sim_to_best = 1.0 - cosine(new_sc.centroid, best_sc.centroid)
            if sim_to_best >= self.sim_threshold(best_sc.vector_count, 1):
                # Add as new subcluster in existing cluster
                self.add_edge(best_sc, new_sc)
                self.clusters[best_cluster_idx].add_subcluster(new_sc)
                self.logger.debug("New subcluster added to existing cluster.")
                return self.clusters[best_cluster_idx].as_dict()
            else:
                # New cluster
                cluster = Cluster(new_sc, logger=self.logger)
                self.clusters.append(cluster)
                self.logger.debug("New cluster created.")
                return cluster.as_dict()

    @staticmethod
    def add_edge(sc1: Subcluster, sc2: Subcluster):
        sc1.connected_subclusters.add(sc2)
        sc2.connected_subclusters.add(sc1)

    def update_edge(self, sc1: Subcluster, sc2: Subcluster) -> bool:
        """Update edge validity between two subclusters based on cosine similarity."""
        sim = 1.0 - cosine(sc1.centroid, sc2.centroid)
        threshold = self.sim_threshold(sc1.vector_count, sc2.vector_count)
        if sim < threshold:
            sc1.connected_subclusters.discard(sc2)
            sc2.connected_subclusters.discard(sc1)
            return False
        else:
            sc1.connected_subclusters.add(sc2)
            sc2.connected_subclusters.add(sc1)
            return True

    def update_cluster(self, cluster_idx: int, subcluster_idx: int):
        """Update cluster edges and merge subclusters if similarity exceeds threshold."""
        updated_sc = self.clusters[cluster_idx].subclusters[subcluster_idx]
        severed_subclusters = []

        for connected_sc in list(updated_sc.connected_subclusters):
            try:
                c_idx = self.clusters[cluster_idx].subclusters.index(connected_sc)
            except ValueError:
                continue

            sim = 1.0 - cosine(updated_sc.centroid, connected_sc.centroid)
            if sim >= self.subcluster_similarity_threshold:
                self.clusters[cluster_idx].merge_subclusters(subcluster_idx, c_idx)
            else:
                if not self.update_edge(updated_sc, connected_sc):
                    severed_subclusters.append(c_idx)

        # Handle severed subclusters
        for sc_idx in sorted(severed_subclusters, reverse=True):
            sc = self.clusters[cluster_idx].subclusters[sc_idx]
            if not sc.connected_subclusters:
                self.clusters[cluster_idx].subclusters.pop(sc_idx)
                self.clusters.append(Cluster(sc, logger=self.logger))

    def get_all_vectors(self) -> List[np.ndarray]:
        """Return all stored vectors."""
        if not self.store_vectors:
            raise RuntimeError("Vectors not stored; cannot retrieve.")
        vectors = []
        for cluster in self.clusters:
            for sc in cluster.subclusters:
                vectors.extend(sc.vectors)
        return vectors

    def sim_threshold(self, k: int, kp: int) -> float:
        """Compute similarity threshold based on cluster sizes."""
        s = (1.0 + 1.0 / k * (1.0 / self.cluster_similarity_threshold**2 - 1.0))
        s *= (1.0 + 1.0 / kp * (1.0 / self.cluster_similarity_threshold**2 - 1.0))
        s = 1.0 / np.sqrt(s)
        s = self.cluster_similarity_threshold**2 + \
            (self.pair_similarity_maximum - self.cluster_similarity_threshold**2) / \
            (1.0 - self.cluster_similarity_threshold**2) * (s - self.cluster_similarity_threshold**2)
        return s
