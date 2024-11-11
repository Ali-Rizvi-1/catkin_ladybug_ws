# Ladybug SDK Installation

Place the Ladybug SDK package (`ladybug-1.20.0.79_modified.deb`) in this directory before building the Docker image.

## Obtaining the SDK

1. Download the SDK package from FLIR's website
2. Copy it to this directory:
   ```bash
   cp /path/to/ladybug-1.20.0.79_modified.deb .
   ```

The Dockerfile will automatically install the SDK during the build process.