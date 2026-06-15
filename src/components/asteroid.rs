pub struct AsteroidMarker {
    /// Index in both the sprite model registry and the GPU instance buffer.
    /// Unique per asteroid so individual models can be edited on destruction.
    pub model_id: u32,
}
