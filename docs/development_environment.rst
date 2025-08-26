Enhanced Development Environment Setup
=====================================

This document describes the enhanced development environment for the Advanced Autonomous Duckietown System, including multi-stage Docker builds, cross-platform development support, and comprehensive testing frameworks.

Overview
--------

The enhanced build system provides three distinct environments:

- **Development**: Full development tools, debugging capabilities, and X11 forwarding for GUI applications
- **Testing**: Comprehensive testing frameworks and coverage tools
- **Production**: Optimized runtime environment with minimal footprint

Multi-Stage Docker Build
------------------------

The Dockerfile now supports multi-stage builds with the following targets:

Development Stage
~~~~~~~~~~~~~~~~~

The development stage includes:

- All base dependencies from the production stage
- Development tools (gdb, valgrind, htop, vim, git, etc.)
- X11 forwarding support for macOS development
- Enhanced debugging capabilities
- Source code mounting support

Build the development image:

.. code-block:: bash

   make build-dev

Testing Stage
~~~~~~~~~~~~~

The testing stage extends the development stage with:

- pytest and testing frameworks
- Code coverage tools
- Test execution utilities
- Continuous integration support

Build the testing image:

.. code-block:: bash

   make build-test

Production Stage
~~~~~~~~~~~~~~~~

The production stage provides:

- Minimal runtime dependencies
- Optimized image size
- Production-ready configuration
- Enhanced security

Build the production image:

.. code-block:: bash

   make build-prod

macOS Development Support
-------------------------

The enhanced system provides comprehensive macOS development support with X11 forwarding for GUI applications.

Prerequisites
~~~~~~~~~~~~~

1. Install XQuartz:

.. code-block:: bash

   brew install --cask xquartz

2. Start XQuartz and enable network connections:
   - Open XQuartz
   - Go to Preferences → Security
   - Check "Allow connections from network clients"

3. Allow localhost connections:

.. code-block:: bash

   xhost +localhost

Development Workflow
~~~~~~~~~~~~~~~~~~~~

1. Setup macOS environment:

.. code-block:: bash

   make macos-setup

2. Start development shell with X11 forwarding:

.. code-block:: bash

   make shell-macos

3. Test GUI applications:

.. code-block:: bash

   # Inside the container
   xclock  # Should display a clock window
   rviz    # ROS visualization tool

Enhanced Build Commands
-----------------------

The Makefile provides comprehensive build and development commands:

Build Commands
~~~~~~~~~~~~~~

.. code-block:: bash

   make build-help      # Show all available commands
   make build-dev       # Build development image
   make build-test      # Build testing image  
   make build-prod      # Build production image
   make build-all       # Build all targets

Development Commands
~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   make shell-dev       # Interactive development shell
   make shell-test      # Interactive testing shell
   make shell-prod      # Interactive production shell
   make shell-mount-dev # Development shell with source mounted

Monitoring Commands
~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   make build-info      # Show build information and logs
   make build-clean     # Clean build artifacts

Advanced Dependencies
---------------------

The enhanced system includes advanced dependencies for autonomous capabilities:

PyTorch and Deep Learning
~~~~~~~~~~~~~~~~~~~~~~~~~

- torch>=1.9.0,<2.0.0
- torchvision>=0.10.0,<1.0.0
- ultralytics>=8.0.0,<9.0.0 (YOLOv5/YOLOv8)

Computer Vision
~~~~~~~~~~~~~~~

- opencv-python>=4.5.0,<5.0.0
- Pillow>=8.0.0,<10.0.0
- scikit-learn>=1.0.0,<2.0.0

Scientific Computing
~~~~~~~~~~~~~~~~~~~~

- scipy>=1.7.0,<2.0.0
- numpy>=1.21.0,<2.0.0
- matplotlib>=3.3.0,<4.0.0

Monitoring and Visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- tensorboard>=2.7.0,<3.0.0

Development Workflow
--------------------

Typical Development Cycle
~~~~~~~~~~~~~~~~~~~~~~~~~

1. **Setup**: Build development environment

.. code-block:: bash

   make build-dev

2. **Develop**: Start development shell with mounted source

.. code-block:: bash

   make shell-mount-dev

3. **Test**: Run tests in testing environment

.. code-block:: bash

   make build-test
   make shell-test
   # Inside container: pytest packages/*/tests/

4. **Deploy**: Build production image

.. code-block:: bash

   make build-prod

Cross-Platform Development
~~~~~~~~~~~~~~~~~~~~~~~~~~

The system supports development on macOS with deployment to Linux robots:

1. **macOS Development**:

.. code-block:: bash

   make macos-setup
   make shell-macos

2. **Linux Testing**:

.. code-block:: bash

   make build-test
   make shell-test

3. **Robot Deployment**:

.. code-block:: bash

   make build-prod
   # Deploy to robot (see deployment documentation)

Build Optimization
------------------

The multi-stage build system provides several optimizations:

Development Optimizations
~~~~~~~~~~~~~~~~~~~~~~~~~

- Comprehensive tooling for debugging and development
- Source code mounting for rapid iteration
- X11 forwarding for GUI applications
- Enhanced logging and monitoring

Testing Optimizations
~~~~~~~~~~~~~~~~~~~~~

- Dedicated testing frameworks
- Code coverage reporting
- Continuous integration support
- Automated test execution

Production Optimizations
~~~~~~~~~~~~~~~~~~~~~~~~

- Minimal image size through layer optimization
- Removed development dependencies
- Cleaned package caches
- Optimized Python bytecode

Troubleshooting
---------------

Common Issues
~~~~~~~~~~~~~

**X11 Forwarding Not Working on macOS**:

1. Ensure XQuartz is running
2. Check network client connections are enabled
3. Verify xhost +localhost was executed
4. Restart Docker if necessary

**Build Failures**:

1. Check Docker daemon is running
2. Verify internet connectivity for dependency downloads
3. Clean build artifacts: ``make build-clean``
4. Check build logs: ``make build-info``

**Performance Issues**:

1. Allocate more memory to Docker
2. Use production image for deployment
3. Monitor resource usage: ``docker stats``

Build Monitoring
~~~~~~~~~~~~~~~~

Monitor build progress and performance:

.. code-block:: bash

   # Show detailed build information
   make build-info
   
   # Monitor container resource usage
   docker stats
   
   # Check build logs
   docker logs <container_id>

The enhanced development environment provides a robust foundation for developing, testing, and deploying the Advanced Autonomous Duckietown System across multiple platforms while maintaining optimal performance and developer productivity.