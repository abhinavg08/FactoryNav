# Publication verification

The portfolio preparation preserves the original robotics packages and recorded benchmark files. It improves presentation, active setup instructions and file selection. The host scenario runner now discovers the simulation container through Compose instead of assuming a checkout-specific container name. Historical logs retain their original names and paths.

Checks completed during publication preparation:

| Check | Outcome |
| :--- | :--- |
| Selected files and reachable Git blobs | No recognized credential patterns, video files or Google Drive links found |
| Local Markdown/image targets | All resolved within the repository |
| Original normal map | Real PNG, 2380 × 1530, generated from the original scene geometry |
| Advanced complex map | Real PNG, 1950 × 1500, generated from the advanced occupancy geometry |
| Headline evidence | 17/17 advanced accepted, 3/3 head-on and 3/3 full missions; zero reported contacts in accepted advanced trials |
| PowerShell sources | Setup and launcher scripts parsed successfully |
| Conda | Discovery, existing-environment reuse and activation verified with the custom installation parameter |
| Host dependencies | NumPy/PyYAML imported; `pip check` passed |
| Geometry checks | All eight Python unit tests passed |
| Compose portability | Base, advanced and advanced/perception configurations resolved successfully from a checkout path containing spaces; loopback port binding retained |
| External documentation | 28 unique documentation/badge URLs responded successfully after outdated links were corrected |
| GitHub Markdown | README accepted by GitHub's rendering API, retaining the two-map table and Mermaid enrichment container |
| Diff integrity | No robotics package or recorded benchmark file modified; no whitespace errors |

No lengthy simulation matrix or fresh network image rebuild was performed for presentation changes. Navigation claims refer to the already-recorded experiments, not new trials. Apt dependencies are not fully frozen, and a future fresh install may require revalidation. See the [setup guide](setup.md) and [detailed limitations](dynamic_navigation_results.md).

Run `python scripts/check_repository.py --history` to repeat the local publication checks. The scanner checks known credential forms and is not a guarantee that every possible secret can be recognized. Runtime environments, downloaded assets, build products and uncurated local reruns are ignored. The demo video is excluded from the repository.
