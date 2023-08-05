

transitions {
    Start.Complete: Submerge,

    Submerge.Unreached: Submerge,
    Submerge.Submerged: Spin,
    Submerge.TimedOut: Spin,

    Spin.Unreached: Spin,
    Spin.TimedOut: SpinFinish,

    SpinFinish.Unreached: SpinFinish,
    SpinFinish.Reached: Suface,
    SpinFinish.TimedOut: Surface,

    Surface.Submerged: Surface,
    Surface.Surfaced: Stop
}
