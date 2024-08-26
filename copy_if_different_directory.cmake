# Get the source and destination directories from the command line arguments
set(SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/driver/amfitrack/resources/")
set(DEST_DIR "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR/resources/")

# Print debug messages
message(STATUS "Copying files from ${SOURCE_DIR} to ${DEST_DIR}")

# Ensure the destination directory exists
file(MAKE_DIRECTORY "${DEST_DIR}")

# Copy the entire directory recursively
file(COPY "${SOURCE_DIR}" DESTINATION "${DEST_DIR}")

# Print a message indicating completion
message(STATUS "Copy completed from ${SOURCE_DIR} to ${DEST_DIR}")