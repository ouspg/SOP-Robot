"""Session-only face identity store.

The store intentionally keeps only anonymous centroids and timing metadata in
memory. It does not persist face embeddings or user identifiers across process
restarts.
"""

from __future__ import annotations

import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Any

import numpy as np

CONVERSATION_THRESHOLD_SECONDS = 15.0
DEFAULT_LOGGER = logging.getLogger(__name__)


def _normalized(vector: np.ndarray) -> np.ndarray | None:
    normalized = np.asarray(vector, dtype=np.float32).reshape(-1)
    norm = float(np.linalg.norm(normalized))
    if norm == 0.0 or not np.isfinite(norm):
        return None
    return normalized / norm


@dataclass
class IdentityRecord:
    centroid: np.ndarray
    id: str = field(default_factory=lambda: str(uuid.uuid4()))
    vector_count: int = 1
    last_seen: float = field(default_factory=time.time)
    conversations: list[dict[str, float]] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.conversations:
            self.conversations.append(
                {
                    "start_time": self.last_seen,
                    "end_time": self.last_seen,
                    "duration": 0.0,
                }
            )

    def add(self, vector: np.ndarray, now: float | None = None) -> None:
        self.vector_count += 1
        weight = 1.0 / self.vector_count
        centroid = self.centroid * (1.0 - weight) + vector * weight
        normalized_centroid = _normalized(centroid)
        if normalized_centroid is not None:
            self.centroid = normalized_centroid
        self.touch(now)

    def touch(self, now: float | None = None) -> None:
        seen_at = time.time() if now is None else now
        if seen_at - self.last_seen <= CONVERSATION_THRESHOLD_SECONDS:
            current = self.conversations[-1]
            current["end_time"] = seen_at
            current["duration"] = current["end_time"] - current["start_time"]
        else:
            self.conversations.append(
                {
                    "start_time": seen_at,
                    "end_time": seen_at,
                    "duration": 0.0,
                }
            )
        self.last_seen = seen_at

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "conversations": [dict(conversation) for conversation in self.conversations],
        }


class SessionIdentityStore:
    """Bounded in-memory identity store for one robot run."""

    def __init__(
        self,
        match_similarity_threshold: float,
        max_identities: int = 8,
        ttl_seconds: float = 300.0,
        logger: logging.Logger | None = None,
    ) -> None:
        self.identities: list[IdentityRecord] = []
        self.match_similarity_threshold = float(match_similarity_threshold)
        self.max_identities = max(0, int(max_identities))
        self.ttl_seconds = max(0.0, float(ttl_seconds))
        self.logger = logger or DEFAULT_LOGGER

    @property
    def clusters(self) -> list[IdentityRecord]:
        """Compatibility alias for old debug/status code."""
        return self.identities

    def predict(self, new_vector: np.ndarray) -> dict[str, Any]:
        vector = _normalized(new_vector)
        if vector is None:
            raise ValueError("Cannot identify a zero or non-finite face embedding.")

        self._prune_expired()
        now = time.time()
        record, similarity = self._best_identity(vector)
        if record is not None and similarity >= self.match_similarity_threshold:
            record.add(vector, now)
            return record.as_dict()

        record = self._create_identity(vector, now)
        self.logger.info("New session face identity created")
        return record.as_dict()

    def touch_cluster(self, cluster_id: str) -> dict[str, Any] | None:
        record = self._find_identity(cluster_id)
        if record is None:
            return None
        record.touch()
        return record.as_dict()

    def update_known_cluster(
        self,
        cluster_id: str,
        new_vector: np.ndarray,
        min_similarity: float | None = None,
    ) -> tuple[dict[str, Any], bool]:
        vector = _normalized(new_vector)
        if vector is None:
            record = self._find_identity(cluster_id)
            if record is None:
                return self.predict(new_vector), True
            record.touch()
            return record.as_dict(), False

        record = self._find_identity(cluster_id)
        if record is None:
            return self.predict(vector), True

        similarity = float(np.dot(record.centroid, vector))
        threshold = (
            self.match_similarity_threshold
            if min_similarity is None
            else float(min_similarity)
        )
        if similarity < threshold:
            record.touch()
            return record.as_dict(), False

        record.add(vector)
        return record.as_dict(), True

    def _create_identity(self, vector: np.ndarray, now: float) -> IdentityRecord:
        record = IdentityRecord(centroid=vector, last_seen=now)
        self.identities.append(record)
        self._enforce_limit()
        return record

    def _find_identity(self, identity_id: str) -> IdentityRecord | None:
        for record in self.identities:
            if record.id == identity_id:
                return record
        return None

    def _best_identity(
        self,
        vector: np.ndarray,
    ) -> tuple[IdentityRecord | None, float]:
        compatible_records = [
            record
            for record in self.identities
            if record.centroid.shape == vector.shape
        ]
        if not compatible_records:
            return None, -np.inf

        centroids = np.vstack([record.centroid for record in compatible_records])
        similarities = centroids @ vector
        best_index = int(np.argmax(similarities))
        return compatible_records[best_index], float(similarities[best_index])

    def _prune_expired(self) -> None:
        if self.ttl_seconds <= 0:
            return

        cutoff = time.time() - self.ttl_seconds
        self.identities = [
            record for record in self.identities if record.last_seen >= cutoff
        ]

    def _enforce_limit(self) -> None:
        if self.max_identities <= 0 or len(self.identities) <= self.max_identities:
            return

        self.identities.sort(key=lambda record: record.last_seen, reverse=True)
        del self.identities[self.max_identities:]
