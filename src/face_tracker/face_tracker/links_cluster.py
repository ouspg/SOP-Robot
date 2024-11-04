"""
Original implementation is from here. Modifications are made to adjust it for SOP-Robot.

https://github.com/QEDan/links_clustering/tree/master

Links online clustering algorithm.

Reference: https://arxiv.org/abs/1801.10123
"""
import logging
import time
import uuid
from typing import List, Dict

import numpy as np
from scipy.spatial.distance import cosine

CONVERSATION_TRESHOLD = 15  # Threshold value to determine that two conversations is same (s)

class Subcluster:
    """Class for subclusters and edges between subclusters."""

    def __init__(self, initial_vector: np.ndarray, store_vectors: bool=False, logger=logging.getLogger()):
        self.logger = logger

        self.vectors = [initial_vector]
        self.centroid = initial_vector
        self.vector_count = 1
        self.store_vectors = store_vectors
        self.connected_subclusters = set()

        # information, when this identity is seen
        now = time.time()
        self.last_seen = now
        self.conversations: List[Dict] = [{
            "start_time": now,
            "end_time": now,
            "duration": 0,
        }]
        self.total_time_on_camera = 0

    def add(self, vector: np.ndarray):
        """Add a new vector to the subcluster, update the centroid."""
        if self.store_vectors:
            self.vectors.append(vector)
        self.vector_count += 1
        if self.centroid is None:
            self.centroid = vector
        else:
            self.centroid = (self.vector_count - 1) / \
                            self.vector_count * self.centroid \
                            + vector / self.vector_count

        # Update time, when seen
        now = time.time()
        if now - self.last_seen <= CONVERSATION_TRESHOLD:
            # conversation is still going
            self.conversations[-1]["end_time"] = now 
            self.conversations[-1]["duration"] = self.conversations[-1]["end_time"] - self.conversations[-1]["start_time"]
            self.total_time_on_camera += now - self.last_seen
        else:
            # conversation is ended -> start new one
            self.conversations.append({
                "start_time": now,
                "end_time": now,
                "duration": 0,
            })

        self.last_seen = now

class Cluster:
    """Class for clusters"""
    def __init__(self, subcluster: Subcluster, logger=logging.getLogger()):
        self.subclusters = [subcluster]
        self.logger = logger

        self.id = str(uuid.uuid4())

    @classmethod
    def from_dict(cls, dict, logger=logging.getLogger()):
        cluster = cls(subcluster=None, logger=logger)
        cluster.id = dict["id"]
        cluster.subclusters = [Subcluster.from_dict(subcluster_dict) for subcluster_dict in dict["subclusters"]]
        return cluster

    def add_subcluster(self, subcluster: Subcluster):
        self.subclusters.append(subcluster)

    def as_dict(self):
        return {
            "id": self.id,
            "conversations": self.calculate_conversation_list(),
        }

    def merge_subclusters(self, sc_idx1, sc_idx2, delete_merged: bool = True):
        """Merge subcluster_merge into self. Update centroids."""
        sc_1 = self.subclusters[sc_idx1]
        sc_2 = self.subclusters[sc_idx2]
        if sc_1.store_vectors:
            sc_1.vectors += sc_2.vectors

        # Update centroid and vector_count
        sc_1.centroid = sc_1.vector_count * sc_1.centroid \
            + sc_2.vector_count \
            * sc_2.centroid
        sc_1.centroid /= sc_1.vector_count + sc_2.vector_count
        sc_1.vector_count += sc_2.vector_count
        try:
            sc_2.connected_subclusters.remove(sc_1)
            sc_1.connected_subclusters.remove(sc_2)
        except KeyError:
            sc_1.logger.warning("Attempted to merge unconnected subclusters. "
                            "Merging anyway.")
        for sc in sc_2.connected_subclusters:
            sc.connected_subclusters.remove(sc_2)
            if sc_1 not in sc.connected_subclusters and sc != sc_1:
                sc.connected_subclusters.update({sc_1})
        sc_1.connected_subclusters.update(sc_2.connected_subclusters)

        # Merge time info
        sc_1.conversations = self.__combine_conversation_lists([sc_1.conversations, sc_2.conversations])

        if delete_merged:
            self.subclusters = self.subclusters[:sc_idx2] \
                + self.subclusters[sc_idx2 + 1:]
            for sc in self.subclusters:
                if sc_2 in sc.connected_subclusters:
                    sc.connected_subclusters.remove(sc_2)

    def calculate_conversation_list(self):
        """
        Calculate conversation times from subclusters conversation info!
        """
        return self.__combine_conversation_lists([sc.conversations for sc in self.subclusters])

    # @staticmethod
    def __combine_conversation_lists(self, conversation_lists: list):
        self.logger.info(f"{conversation_lists=}")
        conversations = []
        # current_conversations = []
        for conversation_list in conversation_lists:
            conversations.extend(conversation_list)
            # current_conversations.append(sc.current_conversation)
        conversations = sorted(conversations, key=lambda x: x["start_time"])
    
        merged = []
        for conv in conversations:
            if not merged or conv["start_time"] >= merged[-1]["end_time"] + CONVERSATION_TRESHOLD:
                # No overlap, add directly
                merged.append(conv)
            else:
                # Overlap, update end time
                merged[-1]["end_time"] = max(merged[-1]["end_time"], conv["end_time"])
                merged[-1]["duration"] = merged[-1]["end_time"] - merged[-1]["start_time"]
        return merged

class LinksCluster:
    """An online clustering algorithm."""
    def __init__(self,
                 cluster_similarity_threshold: float,
                 subcluster_similarity_threshold: float,
                 pair_similarity_maximum: float,
                 store_vectors=False,
                 logger=logging.getLogger()
                 ):
        self.clusters: List[Cluster] = []
        self.cluster_similarity_threshold = cluster_similarity_threshold
        self.subcluster_similarity_threshold = subcluster_similarity_threshold
        self.pair_similarity_maximum = pair_similarity_maximum
        self.store_vectors = store_vectors

        self.logger=logger

    def predict(self, new_vector: np.ndarray) -> dict:
        """Predict a cluster id for new_vector."""
        if len(self.clusters) == 0:
            # Handle first vector
            self.clusters.append(Cluster(Subcluster(new_vector, store_vectors=self.store_vectors)))
            return self.clusters[0].as_dict()

        best_subcluster = None
        best_similarity = -np.inf
        best_subcluster_cluster_id = None
        best_subcluster_id = None
        for cl_idx, cl in enumerate(self.clusters):
            for sc_idx, sc in enumerate(cl.subclusters):
                cossim = 1.0 - cosine(new_vector, sc.centroid)
                if cossim > best_similarity:
                    best_subcluster = sc
                    best_similarity = cossim
                    best_subcluster_cluster_id = cl_idx
                    best_subcluster_id = sc_idx
        if best_similarity >= self.subcluster_similarity_threshold:  # eq. (20)
            # Add to existing subcluster
            best_subcluster.add(new_vector)
            assigned_cluster = self.clusters[best_subcluster_cluster_id]
            self.update_cluster(best_subcluster_cluster_id, best_subcluster_id)
            # assigned_cluster.update(best_subcluster_id)
            self.logger.info("Vector added to excisting sub cluster")
        else:
            # Create new subcluster
            new_subcluster = Subcluster(new_vector, store_vectors=self.store_vectors)
            cossim = 1.0 - cosine(new_subcluster.centroid, best_subcluster.centroid)
            if cossim >= self.sim_threshold(best_subcluster.vector_count, 1):  # eq. (21)
                # New subcluster is part of existing cluster
                self.add_edge(best_subcluster, new_subcluster)
                self.clusters[best_subcluster_cluster_id].add_subcluster(new_subcluster)
                assigned_cluster = self.clusters[best_subcluster_cluster_id]
                self.logger.info("New subcluster created as part of existing cluster")
            else:
                # New subcluster is a new cluster
                assigned_cluster = Cluster(new_subcluster)
                self.clusters.append(assigned_cluster)
                self.logger.info("New subcluster created as a new cluster")
        return assigned_cluster.as_dict()

    @staticmethod
    def add_edge(sc1: Subcluster, sc2: Subcluster):
        """Add an edge between subclusters sc1, and sc2."""
        sc1.connected_subclusters.add(sc2)
        sc2.connected_subclusters.add(sc1)

    def update_edge(self, sc1: Subcluster, sc2: Subcluster):
        """Compare subclusters sc1 and sc2, remove or add an edge depending on cosine similarity.

        Args:
            sc1: Subcluster
                First subcluster to compare
            sc2: Subcluster
                Second subcluster to compare

        Returns:
            bool
                True if the edge is valid
                False if the edge is not valid
        """
        cossim = 1.0 - cosine(sc1.centroid, sc2.centroid)
        threshold = self.sim_threshold(sc1.vector_count, sc2.vector_count)
        if cossim < threshold:
            try:
                sc1.connected_subclusters.remove(sc2)
                sc2.connected_subclusters.remove(sc1)
            except KeyError:
                self.logger.warning("Attempted to update an invalid edge that didn't exist. "
                                "Edge remains nonexistant.")
            return False
        else:
            sc1.connected_subclusters.add(sc2)
            sc2.connected_subclusters.add(sc1)
            return True

    def merge_subclusters(self, cl_idx, sc_idx1, sc_idx2):
        """Merge subclusters with id's sc_idx1 and sc_idx2 of cluster with id cl_idx."""
        sc2 = self.clusters[cl_idx].subclusters[sc_idx2]

        self.clusters[cl_idx].merge_subclusters(sc_idx1, sc_idx2)
        self.update_cluster(cl_idx, sc_idx1)
        # self.clusters[cl_idx].subclusters = self.clusters[cl_idx].subclusters[:sc_idx2] \
        #     + self.clusters[cl_idx].subclusters[sc_idx2 + 1:]
        # for sc in self.clusters[cl_idx].subclusters:
        #     if sc2 in sc.connected_subclusters:
        #         sc.connected_subclusters.remove(sc2)

    def update_cluster(self, cl_idx: int, sc_idx: int):
        """Update cluster

        Subcluster with id sc_idx has been changed, and we want to
        update the parent cluster according to the discussion in
        section 3.4 of the paper.

        Args:
            cl_idx: int
                The index of the cluster to update
            sc_idx: int
                The index of the subcluster that has been changed

        Returns:
            None

        """
        updated_sc = self.clusters[cl_idx].subclusters[sc_idx]
        severed_subclusters = []
        connected_scs = set(updated_sc.connected_subclusters)
        for connected_sc in connected_scs:
            connected_sc_idx = None
            for c_sc_idx, sc in enumerate(self.clusters[cl_idx].subclusters):
                if sc == connected_sc:
                    connected_sc_idx = c_sc_idx
            if connected_sc_idx is None:
                raise ValueError(f"Connected subcluster of {sc_idx} "
                                 f"was not found in cluster list of {cl_idx}.")
            cossim = 1.0 - cosine(updated_sc.centroid, connected_sc.centroid)
            if cossim >= self.subcluster_similarity_threshold:
                self.merge_subclusters(cl_idx, sc_idx, connected_sc_idx)
            else:
                are_connected = self.update_edge(updated_sc, connected_sc)
                if not are_connected:
                    severed_subclusters.append(connected_sc_idx)
        for severed_sc_id in severed_subclusters:
            severed_sc = self.clusters[cl_idx].subclusters[severed_sc_id]
            if len(severed_sc.connected_subclusters) == 0:
                for cluster_sc in self.clusters[cl_idx].subclusters:
                    if cluster_sc != severed_sc:
                        cossim = 1.0 - cosine(cluster_sc.centroid,
                                              severed_sc.centroid)
                        if cossim >= self.sim_threshold(cluster_sc.vector_count,
                                                        severed_sc.vector_count):
                            self.add_edge(cluster_sc, severed_sc)
            if len(severed_sc.connected_subclusters) == 0:
                self.clusters[cl_idx].subclusters = self.clusters[cl_idx].subclusters[:severed_sc_id] \
                    + self.clusters[cl_idx].subclusters[severed_sc_id + 1:]
                self.clusters.append(Cluster(severed_sc))

    def get_all_vectors(self):
        """Return all stored vectors from entire history.

        Returns:
            list
                list of vectors

        Raises:
            RuntimeError
                if self.store_vectors is False (i.e. there are no stored vectors)
        """
        if not self.store_vectors:
            raise RuntimeError("Vectors were not stored, so can't be collected")
        all_vectors = []
        for cl in self.clusters:
            for scl in cl.subclusters:
                all_vectors += scl.vectors
        return all_vectors

    def sim_threshold(self, k: int, kp: int) -> float:
        """Compute the similarity threshold.

        This is based on equations (16) and (24) of the paper.

        Args:
            k: int
                The number of vectors in a cluster or subcluster
            kp: int
                k-prime in the paper, the number of vectors in another
                cluster or subcluster

        Returns:
            float
                The similarity threshold for inclusion in a cluster or subcluster.
        """
        s = (1.0 + 1.0 / k * (1.0 / self.cluster_similarity_threshold ** 2 - 1.0))
        s *= (1.0 + 1.0 / kp * (1.0 / self.cluster_similarity_threshold ** 2 - 1.0))
        s = 1.0 / np.sqrt(s)  # eq. (16)
        s = self.cluster_similarity_threshold ** 2 \
            + (self.pair_similarity_maximum - self.cluster_similarity_threshold ** 2) \
            / (1.0 - self.cluster_similarity_threshold ** 2) \
            * (s - self.cluster_similarity_threshold ** 2)  # eq. (24)
        return s
