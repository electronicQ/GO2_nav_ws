# Quatro source provenance

- Source: https://github.com/engcang/Quatro
- Commit: d27109bd6a1798e9cf2e0c2a4daac6af16e7bc23
- Original GPL-3.0 license is preserved in License.
- FPFH/matcher files retain MIT-SPARK/TEASER++ attribution.
- The ROS1 catkin wrapper is not built. The core is compiled by ament with
  TEASER++ registration (which must include ROTATION_ESTIMATION_ALGORITHM::QUATRO).
- Matching follows the SC-QN route: one saved candidate cloud, FPFH + Quatro
  coarse alignment, then Nano-GICP. SC proposals use Quatro/GICP checks and the basic global-transform commit gates.
  Established alignments additionally trim SC candidates by predicted map position;
  DIRECT retains its own final/fresh-scan plane audits. No upstream ROS publishing code is copied.
- This optional component is not BSD-only; retain all component licenses.
- Local fix: fpfh.cc includes its own quatro/fpfh.h, avoiding a class-layout
  mismatch with newer TEASER++ headers of the same class name.
- FPFH explicitly honors the adapter's thread-local OpenMP budget instead of
  defaulting to every hardware core. No OpenMP flags propagate to the LIO target.
- Local TEASER++ build: MIT-SPARK/TEASER-plusplus commit
  52a9c52ee7d4c838c5e8a75458c33178be5bfb70; original MIT license in TEASER_LICENSE.
  PMC a2dfd612a501bca83c47206255dbbff619481f97,
  Spectra 5c4fb1de050847988faaaaa50f60e7d3d5f16143,
  tinyply c9bb690dfe5e9105961e9e28120c48c9ae084bc6.
