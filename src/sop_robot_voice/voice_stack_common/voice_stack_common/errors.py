"""Shared exception types for the SOP Robot voice stack."""


class AudioDependencyError(RuntimeError):
    """Raised when required audio runtime dependencies are missing."""
