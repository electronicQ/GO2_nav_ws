# Scan Context source provenance

- Source: https://github.com/engcang/scancontext_tro
- Commit: c8ef5b496a159cdfd7fa4761121178f25cd0a6bb
- Used by the reference https://github.com/engcang/FAST-LIO-Localization-SC-QN.
- Authors: Giseop Kim and collaborators, KAIST and NAVER LABS; attribution,
  publications, and CC BY-NC-SA 4.0 terms are preserved in README.md.
- Bundled nanoflann headers retain their original copyright/license notices.
- Local changes: remove unused ROS conversion include; make the common height
  offset configurable. The adapter uses the upstream ring-key KD-tree and
  cyclic cosine distance, but searches the entire immutable history bank without
  the online loop detector's recent-frame exclusion. Query descriptors never enter
  the history bank. History and query are gravity-aligned identically.
- This optional component is not BSD-only and must not be represented as such.
