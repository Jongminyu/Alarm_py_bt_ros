# CHANGELOG.md

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/)  
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added
- Initial extraction for standalone repository as `py_bt_ros`:
  - Sourced from: https://github.com/inmo-jang/space-simulator  
  - Extracted using:
    ```
    git filter-repo --paths-from-file extract/paths.txt --force
    ```
  - Included modules:
    ```
    scenarios/features/ros/
    modules/base_agent.py
    modules/base_bt_nodes.py
    modules/base_env.py
    modules/base_task.py
    modules/bt_constructor.py
    modules/bt_visualiser.py
    modules/utils.py
    main.py
    requirements.txt
    ```

### Changed
- N/A

### Fixed
- N/A
