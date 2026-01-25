# Syncing JoeboyC2 WLED Fork with the Upstream Repository

This guide explains how to manually sync the fork with the upstream WLED repository's latest release, while also keeping the custom changes in the JoeboyC2_Mods_V3 branch. These steps have also been automated using scripts and a GitHub workflow.

---

## **Automated Sync (Recommended)**

### **Using PowerShell Script (Windows)**

The enhanced sync script provides multiple modes for safe synchronization:

```powershell
# Analyze differences WITHOUT making changes (safe to run anytime)
.\sync_fork.ps1 -CheckOnly

# Test the sync process without actually changing your repo
.\sync_fork.ps1 -TestMode

# Remove unwanted core changes, keep only usermods and config
.\sync_fork.ps1 -Cleanup

# Perform actual sync
.\sync_fork.ps1

# Combine modes for more control
.\sync_fork.ps1 -CheckOnly -Verbose    # Detailed analysis
.\sync_fork.ps1 -TestMode -Cleanup     # Test cleanup mode
```

**Features of the enhanced script:**

- ✅ Pre-merge analysis of all file differences
- ✅ Automatic detection of high divergence (warns if >20 files differ)
- ✅ Cleanup mode to remove unwanted accumulated changes
- ✅ Check-only mode for safe analysis
- ✅ Detailed conflict reporting with resolution options
- ✅ Test mode to preview changes before committing

### **Using Linux/Mac/Git Bash:**

```bash
chmod +x sync_fork_bash.sh
./sync_fork_bash.sh
```

---

## **Recommended Sync Workflow**

### **1. Safe Analysis (Always Safe)**

```powershell
.\sync_fork.ps1 -CheckOnly -Verbose
```

This shows:

- What files will change
- How many files differ
- Whether cleanup is recommended

### **2. Test the Sync (Safe)**

```powershell
.\sync_fork.ps1 -TestMode
```

Previews all changes without modifying your repo.

### **3. Clean Up If Needed (Safe)**

If CheckOnly shows many unwanted core files changed:

```powershell
.\sync_fork.ps1 -Cleanup -TestMode    # Preview cleanup
.\sync_fork.ps1 -Cleanup               # Actually clean up
```

### **4. Perform the Sync**

```powershell
.\sync_fork.ps1
```

---

## **Manual Sync Steps - Linux/Mac/Git Bash**

### **Step 1: Add the Original Repository as Upstream**

```bash
# Check the current setup remotes
git remote -v

# Add the upstream remote if not already added
git remote add upstream https://github.com/Aircoookie/WLED.git
```

### **Step 2: Reset Main Branch to Latest Release**

```bash
# Fetch the latest changes and tags from upstream
git fetch upstream --tags --force

# Get the most recent stable release tag (excluding betas and pre-releases)
latest_tag=$(git tag -l | grep -E "^v[0-9]+\.[0-9]+\.[0-9]+$" | sort -V | tail -n 1)
echo "Latest tag: $latest_tag"

# Switch to main branch
git checkout main

# Reset main to match the latest release tag exactly
git reset --hard $latest_tag

# Force push the updated main branch since we're rewriting history
git push origin main --force
```

### **Step 3: Merge Updates into Custom Branch**

```bash
# Switch to custom branch (JoeboyC2_Mods_V3)
git checkout JoeboyC2_Mods_V3

# Merge the updated main branch into custom branch
git merge main

# If conflicts occur, resolve them, then:
git add .
git commit -m "Merged updates from latest release"

# Push the changes to your custom branch
git push origin JoeboyC2_Mods_V3
```

### **Step 4: Test Customizations**

Test your customizations with the new code to ensure everything works as expected.

---

## **Manual Sync Steps - Windows PowerShell**

### **Manual Step 1: Add the Original Repository as Upstream**

```powershell
# Check the current setup remotes
git remote -v

# Add the upstream remote if not already added
git remote add upstream https://github.com/Aircoookie/WLED.git
```

### **Manual Step 2: Reset Main Branch to Latest Release**

```powershell
# Fetch the latest changes and tags from upstream
git fetch upstream --tags --force

# Get the most recent stable release tag (excluding betas and pre-releases)
$tags = git tag -l | Where-Object { $_ -match "^v\d+\.\d+\.\d+$" }
$latest_tag = $tags | Sort-Object { [Version]($_ -replace 'v', '') } | Select-Object -Last 1
Write-Host "Latest tag: $latest_tag"

# Switch to main branch
git checkout main

# Reset main to match the latest release tag exactly
git reset --hard $latest_tag

# Force push the updated main branch since we're rewriting history
git push origin main --force
```

### **Manual Step 3: Merge Updates into Custom Branch**

```powershell
# Switch to custom branch (JoeboyC2_Mods_V3)
git checkout JoeboyC2_Mods_V3

# Merge the updated main branch into custom branch
git merge main

# If conflicts occur, resolve them, then:
git add .
git commit -m "Merged updates from latest release"

# Push the changes to your custom branch
git push origin JoeboyC2_Mods_V3
```

### **Manual Step 4: Test Customizations**

Test your customizations with the new code to ensure everything works as expected.

---

## **Handling Merge Conflicts**

If conflicts occur during merge, the script will display them. You have several options:

### **Option 1: Resolve Conflicts Manually**

1. Open conflicted files in your editor
2. Find `<<<<<<<` conflict markers
3. Choose which version to keep
4. Remove conflict markers
5. Stage and commit: `git add . && git commit -m "Resolved merge conflicts"`

### **Option 2: Keep Your Custom Changes**

```bash
# Keep your version of specific files
git checkout --ours <filename>
git add <filename>
```

### **Option 3: Keep Upstream Changes**

```bash
# Keep upstream version of specific files
git checkout --theirs <filename>
git add <filename>
```

### **Option 4: Use Cleanup Mode**

```powershell
# Abort the merge first
git merge --abort

# Then run cleanup to remove unwanted core changes
.\sync_fork.ps1 -Cleanup

# Then retry the sync
.\sync_fork.ps1
```

---

## **GitHub Actions Workflow**

This repository includes a GitHub Actions workflow to automate the process of syncing the JoeboyC2 fork with the latest release. The workflow is triggered manually and performs the following steps automatically:

1. Fetches the latest release tag from the upstream repository.
2. Resets the `main` branch to exactly match the latest release tag.
3. Merges the updated `main` branch into your custom branch (`JoeboyC2_Mods_V3`).

To trigger the workflow:

1. Go to the **Actions** tab in this repository.
2. Select the **Sync Upstream** workflow.
3. Click on **Run workflow** and follow the prompts.

You can find the workflow definition in `.github/workflows/sync_upstream.yml`.

---

## **Important Notes**

- **Always run `-CheckOnly` first** to understand what will change
- **Test your customizations** after syncing to ensure everything works with the new code
- The force push in Step 2 rewrites history on the main branch. This is intentional to keep it in sync with upstream.
- If merge conflicts occur, carefully review what's changing before accepting
- Keep only your essential customizations (usermods, platformio_override.ini, and the integration files in wled00/)
- Use `-Cleanup` mode if accumulated changes have made the branch divergent
- Always make sure you're in the correct directory (your repository root) before running these commands
- **Windows users:** If you encounter PowerShell execution policy errors, use:

  ```powershell
  PowerShell -ExecutionPolicy Bypass -File .\sync_fork.ps1
  ```

  or use Git Bash instead
