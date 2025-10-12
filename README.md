🛠️ Setup: Changing User Directories
This robotics package often relies on absolute file paths (e.g., in configuration, launch, or setup files) that include the previous developer's home directory (e.g., /home/old-user-name/...).

Before building or running this package on a new machine, you must replace all occurrences of the previous user's name with your current device's username.

1. Identify Placeholders
Placeholder

Meaning

Example Value

old-user-name

The username present in the repository's files.

omar-magdy

new-user-name

Your actual Linux username.

omar-magdy

2. Command to Replace File Content
Navigate to the root of your workspace (e.g., your colcon or catkin workspace) and execute the sed command below. This command recursively replaces the old-user-name with your new-user-name across the contents of all files.

⚠️ Warning: This command performs in-place editing, modifying the files directly. Proceed with caution.

# General Syntax:
find . -type f -exec sed -i 's/old-user-name/new-user-name/g' {} +

# Example Implementation (Replace the usernames below with your specific values):
# find . -type f -exec sed -i 's/omar-magdy/omar-magdy/g' {} +

3. Verify Changes (Optional Check)
After running the replacement command, use grep to quickly confirm if any instances of the original username remain in any files:

# Search for any remaining instances of the old username:
grep -r 'old-user-name' .

If this command returns no output, the replacement was successful.

If it returns file paths and lines of code, you may need to manually inspect and correct those files.