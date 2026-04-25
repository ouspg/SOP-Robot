import time

import numpy as np

from face_tracker.identity_store import SessionIdentityStore


def test_same_embedding_reuses_session_identity():
    store = SessionIdentityStore(match_similarity_threshold=0.8)

    first = store.predict(np.array([1.0, 0.0, 0.0]))
    second = store.predict(np.array([0.99, 0.01, 0.0]))

    assert first["id"] == second["id"]
    assert len(store.identities) == 1


def test_store_caps_session_identities():
    store = SessionIdentityStore(match_similarity_threshold=0.95, max_identities=2)

    first = store.predict(np.array([1.0, 0.0, 0.0]))
    store.predict(np.array([0.0, 1.0, 0.0]))
    store.predict(np.array([0.0, 0.0, 1.0]))

    assert len(store.identities) == 2
    assert all(record.id != first["id"] for record in store.identities)


def test_store_expires_stale_identities():
    store = SessionIdentityStore(match_similarity_threshold=0.8, ttl_seconds=1.0)

    first = store.predict(np.array([1.0, 0.0, 0.0]))
    store.identities[0].last_seen = time.time() - 2.0
    second = store.predict(np.array([1.0, 0.0, 0.0]))

    assert first["id"] != second["id"]
    assert len(store.identities) == 1


def test_new_store_starts_without_persisted_identities():
    first_store = SessionIdentityStore(match_similarity_threshold=0.8)
    first_store.predict(np.array([1.0, 0.0, 0.0]))

    second_store = SessionIdentityStore(match_similarity_threshold=0.8)

    assert second_store.identities == []
