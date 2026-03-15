# Firefly ROS2 Wrapper

## Requirements: Spinnaker SDK

This package requires the **Spinnaker SDK** from Teledyne Vision Solutions.

### 1. Download

Go to the [Spinnaker SDK Downloads page](https://www.teledynevisionsolutions.com/support/support-center/software-firmware-downloads/iis/spinnaker-sdk-download/spinnaker-sdk--download-files/?pn=Spinnaker+SDK&vn=Spinnaker+SDK) and download the appropriate version for your system. You may need to create a free account.

I am using version: 
4.2 Linux | Jan 10, 2025 | 4.2.0.46

### 2. Install

```bash
# Extract the archive
tar -xvf spinnaker-*.tar.gz

# Run the installer from inside the extracted folder
cd spinnaker-*/
./install_spinnaker.sh
```

Follow the prompts during installation. Make sure to:
- **Add udev rules** for your current user (required for USB camera access)
- **Approve the USB-FS memory increase** to 1000 MB at startup (required for high-bandwidth cameras)

The following optional components are **not needed**:
- Prebuilt samples (recommended)
- GenTL Producer (recommended)
- GigEVision cameras

### 3. Verify Installation

```bash
find /usr/lib /usr/local/lib -name "libSpinnaker*" 2>/dev/null
```

If a library path is returned, the SDK is installed correctly.
