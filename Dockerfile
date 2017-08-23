FROM ros:kinetic

RUN apt-get update && apt-get install -y \
    ros-kinetic-mavros \
    ros-kinetic-mavros-extras

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

