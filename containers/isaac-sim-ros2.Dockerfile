ARG ISAAC_IMAGE=nvcr.io/nvidia/isaac-sim:5.1.0
FROM ${ISAAC_IMAGE}

ARG ISAAC_IMAGE
ARG ISAAC_BASE_IMAGE_ID=unknown
ARG ISAAC_ROS_DISTRO=auto
ARG ISAAC_ROS_INSTALL_VARIANT=ros-base
ARG ISAAC_ROS_DOCKERFILE_HASH=unknown

SHELL ["/bin/bash", "-lc"]
USER root

RUN set -euo pipefail; \
    . /etc/os-release; \
    case "${VERSION_ID}" in \
      22.04) expected_distro="humble" ;; \
      24.04) expected_distro="jazzy" ;; \
      *) echo "Unsupported Ubuntu version for ROS 2 auto selection: ${VERSION_ID}" >&2; exit 1 ;; \
    esac; \
    requested_distro="${ISAAC_ROS_DISTRO}"; \
    if [[ "${requested_distro}" == "auto" ]]; then \
      requested_distro="${expected_distro}"; \
    elif [[ "${requested_distro}" != "${expected_distro}" ]]; then \
      echo "ISAAC_ROS_DISTRO=${requested_distro} is incompatible with Ubuntu ${VERSION_ID}; use ${expected_distro} or auto." >&2; \
      exit 1; \
    fi; \
    case "${requested_distro}" in humble|jazzy) ;; *) echo "Unsupported ISAAC_ROS_DISTRO=${requested_distro}" >&2; exit 1 ;; esac; \
    case "${ISAAC_ROS_INSTALL_VARIANT}" in ros-base|desktop) ;; *) echo "Unsupported ISAAC_ROS_INSTALL_VARIANT=${ISAAC_ROS_INSTALL_VARIANT}" >&2; exit 1 ;; esac; \
    printf '%s\n' "${requested_distro}" >/etc/isaac-projects-ros-distro; \
    printf '%s\n' "${ISAAC_ROS_INSTALL_VARIANT}" >/etc/isaac-projects-ros-install-variant; \
    apt-get update; \
    apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release software-properties-common locales; \
    locale-gen en_US en_US.UTF-8; \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
    add-apt-repository -y universe; \
    apt-get update; \
    ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F 'tag_name' | awk -F'"' '{print $4}')"; \
    [[ -n "${ros_apt_version}" ]]; \
    curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${UBUNTU_CODENAME:-${VERSION_CODENAME}}_all.deb"; \
    dpkg -i /tmp/ros2-apt-source.deb; \
    rm -f /tmp/ros2-apt-source.deb; \
    apt-get update; \
    ros_packages=( \
      "ros-${requested_distro}-${ISAAC_ROS_INSTALL_VARIANT}" \
      "ros-${requested_distro}-rmw-fastrtps-cpp" \
      "ros-${requested_distro}-rmw-implementation" \
    ); \
    for optional_package in \
      "ros-${requested_distro}-rmw-cyclonedds-cpp" \
      "ros-${requested_distro}-demo-nodes-cpp" \
      "ros-${requested_distro}-demo-nodes-py"; do \
      if apt-cache show "${optional_package}" >/dev/null 2>&1; then \
        ros_packages+=("${optional_package}"); \
      fi; \
    done; \
    apt-get install -y --no-install-recommends \
      git \
      cmake \
      build-essential \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-dev-tools \
      "${ros_packages[@]}"; \
    rosdep init >/dev/null 2>&1 || true; \
    rosdep update || true; \
    rm -rf /var/lib/apt/lists/*

RUN set -euo pipefail; \
    ros_distro="$(cat /etc/isaac-projects-ros-distro)"; \
    { \
      printf '%s\n' '# Source ROS 2 for normal shell work. Isaac Sim app launches use non-login'; \
      printf '%s\n' "# shells so the system ROS Python environment does not interfere with Isaac's"; \
      printf '%s\n' '# embedded Python runtime.'; \
      printf '%s\n' 'export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"'; \
      printf '%s\n' "if [ \"\${ISAAC_PROJECTS_NO_AUTO_ROS:-0}\" != \"1\" ] && [ \"\${ROS_DISTRO:-}\" != \"${ros_distro}\" ] && [ -f \"/opt/ros/${ros_distro}/setup.sh\" ]; then"; \
      printf '%s\n' '  __isaac_projects_nounset_was_enabled=0'; \
      printf '%s\n' '  case $- in *u*) __isaac_projects_nounset_was_enabled=1; set +u ;; esac'; \
      printf '%s\n' "  . \"/opt/ros/${ros_distro}/setup.sh\""; \
      printf '%s\n' '  if [ "$__isaac_projects_nounset_was_enabled" = "1" ]; then set -u; fi'; \
      printf '%s\n' '  unset __isaac_projects_nounset_was_enabled'; \
      printf '%s\n' 'fi'; \
    } >/etc/profile.d/isaac-projects-ros2.sh; \
    chmod 0644 /etc/profile.d/isaac-projects-ros2.sh; \
    block_begin="# >>> isaac-projects ROS 2 setup >>>"; \
    block_end="# <<< isaac-projects ROS 2 setup <<<"; \
    block_body="export ROS_DOMAIN_ID=\"\${ROS_DOMAIN_ID:-0}\"\nif [ \"\${ISAAC_PROJECTS_NO_AUTO_ROS:-0}\" != \"1\" ] && [ \"\${ROS_DISTRO:-}\" != \"${ros_distro}\" ] && [ -f \"/opt/ros/${ros_distro}/setup.bash\" ]; then\n  __isaac_projects_nounset_was_enabled=0\n  case \$- in *u*) __isaac_projects_nounset_was_enabled=1; set +u ;; esac\n  source \"/opt/ros/${ros_distro}/setup.bash\"\n  if [ \"\$__isaac_projects_nounset_was_enabled\" = \"1\" ]; then set -u; fi\n  unset __isaac_projects_nounset_was_enabled\nfi"; \
    for shell_file in /etc/bash.bashrc /root/.bashrc /etc/skel/.bashrc; do \
      touch "${shell_file}"; \
      tmp_file="$(mktemp)"; \
      awk -v begin="${block_begin}" -v end="${block_end}" '$0 == begin { skip=1; next } $0 == end { skip=0; next } !skip { print }' "${shell_file}" >"${tmp_file}"; \
      { printf '\n%s\n' "${block_begin}"; printf '%b\n' "${block_body}"; printf '%s\n' "${block_end}"; } >>"${tmp_file}"; \
      install -m 0644 "${tmp_file}" "${shell_file}"; \
      rm -f "${tmp_file}"; \
    done

LABEL rice.isaacsim.ros2.enabled="true" \
      rice.isaacsim.ros2.base_image="${ISAAC_IMAGE}" \
      rice.isaacsim.ros2.base_image_id="${ISAAC_BASE_IMAGE_ID}" \
      rice.isaacsim.ros2.distro="${ISAAC_ROS_DISTRO}" \
      rice.isaacsim.ros2.install_variant="${ISAAC_ROS_INSTALL_VARIANT}" \
      rice.isaacsim.ros2.dockerfile_hash="${ISAAC_ROS_DOCKERFILE_HASH}"

WORKDIR /workspace/isaac-projects
