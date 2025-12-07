# Syncing JoeboyC2 WLED Fork with the Upstream Repository

This guide explains how to manually sync the fork with the upstream WLED repository's latest release, while also keeping the custom changes in the JoeboyC2_Mods_V3 branch. These steps have also been automated using scripts and a GitHub workflow.

---

## **Automated Sync (Recommended)**

### **Using Scripts**

Two cross-platform scripts are available to automate the sync process:

#### **Linux/Mac/Git Bash (Windows):**
```bash
chmod +x sync-fork.sh
./sync-fork.sh
```

#### **Windows PowerShell:**
```powershell
PowerShell -ExecutionPolicy Bypass -File .\sync_fork.ps1
```

Both scripts will:
- Check and add the upstream remote if needed
- Fetch the latest stable release tag
- Reset your main branch to match the upstream release
- Merge updates into your custom branch (JoeboyC2_Mods_V3)
- Prompt for confirmation before force pushing

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
git fetch upstream --tags

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

### **Step 1: Add the Original Repository as Upstream**
```powershell
# Check the current setup remotes
git remote -v

# Add the upstream remote if not already added
git remote add upstream https://github.com/Aircoookie/WLED.git
```

### **Step 2: Reset Main Branch to Latest Release**
```powershell
# Fetch the latest changes and tags from upstream
git fetch upstream --tags

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

### **Step 3: Merge Updates into Custom Branch**
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

### **Step 4: Test Customizations**
Test your customizations with the new code to ensure everything works as expected.

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

- **Test your customizations** after syncing to ensure everything works with the new code.
- The force push in Step 2 rewrites history on the main branch. This is intentional to keep it in sync with upstream.
- If merge conflicts occur in Step 3, you'll need to resolve them manually before pushing.
- Always make sure you're in the correct directory (your repository root) before running these commands.
- **Windows users:** If you encounter PowerShell execution policy errors, use the automated script command shown above which bypasses the policy, or use Git Bash instead.