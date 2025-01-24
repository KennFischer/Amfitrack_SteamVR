# Get the source and destination directories from the command line arguments
set(SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/driver/amfitrack/")
set(DEST_DIR "${CMAKE_CURRENT_LIST_DIR}/release/Amfitrack_SteamVR/")

# Print debug messages
message(STATUS "Copying files from ${SOURCE_DIR} to ${DEST_DIR}")

# Ensure the destination directory exists
file(MAKE_DIRECTORY "${DEST_DIR}")

# Copy the entire directory recursively
file(COPY "${SOURCE_DIR}" DESTINATION "${DEST_DIR}")

# Print a message indicating completion
message(STATUS "Copy completed from ${SOURCE_DIR} to ${DEST_DIR}")