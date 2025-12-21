# Syncing JoeboyC2 WLED Fork with the Upstream WLED Repository
# 
# Features:
# - Validates git state before operations
# - Pre-merge analysis of file differences
# - Proper merge conflict detection
# - Safety checks before force push
# - Automatic default branch detection
# - Comprehensive error handling
# - Cleanup mode to remove unwanted changes
# - Check-only mode for safe analysis
# 
# Usage:
#   .\sync_fork.ps1                    # Run normally
#   .\sync_fork.ps1 -TestMode          # Test without making changes
#   .\sync_fork.ps1 -CheckOnly         # Analyze differences without syncing
#   .\sync_fork.ps1 -Cleanup           # Remove unwanted core changes, keep only usermods
#   .\sync_fork.ps1 -Verbose           # Show debug output
#
# Examples:
#   .\sync_fork.ps1 -CheckOnly -Verbose    # Safe analysis with details
#   .\sync_fork.ps1 -TestMode -Cleanup     # Test cleanup mode
#   .\sync_fork.ps1 -Cleanup               # Actually clean up unwanted changes

param(
    [switch]$TestMode = $false,
    [switch]$Verbose = $false,
    [switch]$CheckOnly = $false,
    [switch]$Cleanup = $false
)

$ErrorActionPreference = "Stop"

$UPSTREAM_REPO = "https://github.com/Aircoookie/WLED.git"
$CUSTOM_BRANCH = "JoeboyC2_Mods_V3"

# Helper function for command execution with error checking
function Invoke-GitCommand {
    param(
        [string]$Command,
        [string]$Description = ""
    )
    
    if ($Verbose) {
        Write-Host "  [DEBUG] Running: $Command" -ForegroundColor Gray
    }
    
    if ($TestMode) {
        Write-Host "  [TEST MODE] Would execute: $Command" -ForegroundColor Cyan
        return
    }
    
    Invoke-Expression $Command
    if ($LASTEXITCODE -ne 0) {
        if ($Description) {
            Write-Host "[ERROR] $Description failed (exit code: $LASTEXITCODE)" -ForegroundColor Red
        } else {
            Write-Host "[ERROR] Command failed: $Command (exit code: $LASTEXITCODE)" -ForegroundColor Red
        }
        exit 1
    }
}

# Helper function to check if working directory is clean
function Test-GitWorkingDirectory {
    $status = git status --porcelain
    if ($status) {
        Write-Host "[ERROR] Working directory has uncommitted changes:" -ForegroundColor Red
        Write-Host $status
        Write-Host ""
        Write-Host "Please commit or stash your changes before syncing." -ForegroundColor Yellow
        exit 1
    }
}

# Helper function to get default branch
function Get-DefaultBranch {
    $defaultBranch = git symbolic-ref refs/remotes/origin/HEAD | Split-Path -Leaf
    if (-not $defaultBranch) {
        # Fallback: check if main or master exists
        if (git rev-parse --verify origin/main 2>$null) {
            return "main"
        } elseif (git rev-parse --verify origin/master 2>$null) {
            return "master"
        } else {
            Write-Host "[ERROR] Could not determine default branch" -ForegroundColor Red
            exit 1
        }
    }
    return $defaultBranch
}

# Helper function to analyze file differences between branches
function Show-BranchDifferences {
    param(
        [string]$Branch1,
        [string]$Branch2,
        [string]$Description
    )
    
    Write-Host ""
    Write-Host "=== $Description ===" -ForegroundColor Cyan
    
    $diffFiles = @(git diff --name-only "$Branch1..$Branch2" 2>$null)
    
    if ($diffFiles.Count -eq 0) {
        Write-Host "[OK] No differences found" -ForegroundColor Green
        return 0
    }
    
    Write-Host "Files that differ ($($diffFiles.Count) total):" -ForegroundColor Yellow
    $diffFiles | ForEach-Object { Write-Host "  - $_" }
    
    # Show statistics
    $stats = git diff --stat "$Branch1..$Branch2" 2>$null
    Write-Host ""
    Write-Host "Statistics:" -ForegroundColor Cyan
    $stats | Select-Object -Last 1 | ForEach-Object { Write-Host "  $_" }
    
    return $diffFiles.Count
}

# Helper function to cleanup unwanted core changes
function Invoke-CleanupUnwantedChanges {
    param(
        [string]$DefaultBranch
    )
    
    Write-Host ""
    Write-Host "=== Cleanup Mode ===" -ForegroundColor Cyan
    Write-Host "This will revert core files to baseline, keeping only:" -ForegroundColor Yellow
    Write-Host "  • usermods/JoeboyC2_*/" -ForegroundColor White
    Write-Host "  • platformio_override.ini" -ForegroundColor White
    Write-Host "  • wled00/const.h (usermod defines)" -ForegroundColor White
    Write-Host "  • wled00/usermods_list.cpp (usermod registration)" -ForegroundColor White
    Write-Host ""
    
    $filesToRevert = @(
        "wled00/FX.cpp",
        "wled00/FX.h",
        "wled00/FX_fcn.cpp",
        "wled00/bus_manager.cpp",
        "wled00/bus_manager.h",
        "wled00/bus_wrapper.h",
        "wled00/cfg.cpp",
        "wled00/wled.cpp",
        "wled00/wled.h",
        "wled00/mqtt.cpp",
        "wled00/led.cpp",
        "wled00/json.cpp",
        "wled00/set.cpp",
        "wled00/udp.cpp",
        "wled00/util.cpp",
        "wled00/xml.cpp",
        "wled00/src/dependencies/time/DateStrings.cpp"
    )
    
    $filesFound = 0
    foreach ($file in $filesToRevert) {
        $diff = git diff --quiet $DefaultBranch..HEAD -- $file 2>$null
        if ($LASTEXITCODE -eq 1) {
            Write-Host "Reverting: $file" -ForegroundColor Yellow
            if (-not $TestMode) {
                git checkout $DefaultBranch -- $file
                if ($LASTEXITCODE -ne 0) {
                    Write-Host "[WARN] Failed to revert $file" -ForegroundColor Yellow
                } else {
                    $filesFound++
                }
            } else {
                Write-Host "  [TEST MODE] Would revert: $file" -ForegroundColor Cyan
                $filesFound++
            }
        }
    }
    
    if ($filesFound -gt 0) {
        Write-Host ""
        Write-Host "[OK] Reverted $filesFound files to baseline" -ForegroundColor Green
        if (-not $TestMode) {
            Write-Host "Review changes with: git diff --stat" -ForegroundColor Cyan
            Write-Host "Commit with: git add . && git commit -m 'Cleanup: revert unwanted core changes'" -ForegroundColor Cyan
        }
    } else {
        Write-Host "[OK] No unwanted core changes found" -ForegroundColor Green
    }
}

Write-Host "=== Syncing Fork with Upstream WLED Repository ===" -ForegroundColor Cyan
if ($TestMode) {
    Write-Host "[TEST MODE] - No changes will be made" -ForegroundColor Cyan
}
if ($CheckOnly) {
    Write-Host "[CHECK ONLY] - Analyzing without syncing" -ForegroundColor Cyan
}
if ($Cleanup) {
    Write-Host "[CLEANUP MODE] - Will remove unwanted core changes" -ForegroundColor Cyan
}
Write-Host ""

# Handle cleanup mode
if ($Cleanup) {
    Write-Host "Step 0: Validating git state..." -ForegroundColor Yellow
    Test-GitWorkingDirectory
    Write-Host "[OK] Working directory is clean" -ForegroundColor Green
    
    Invoke-CleanupUnwantedChanges $CUSTOM_BRANCH
    
    if (-not $TestMode) {
        Write-Host ""
        $confirm = Read-Host "Commit cleanup changes? (y/N)"
        if ($confirm -eq 'y' -or $confirm -eq 'Y') {
            git add .
            git commit -m "Cleanup: revert unwanted core changes"
            Write-Host "[OK] Changes committed" -ForegroundColor Green
            
            $pushConfirm = Read-Host "Push to origin/$CUSTOM_BRANCH? (y/N)"
            if ($pushConfirm -eq 'y' -or $pushConfirm -eq 'Y') {
                git push origin $CUSTOM_BRANCH
                Write-Host "[OK] Changes pushed" -ForegroundColor Green
            }
        }
    }
    exit 0
}

# Handle check-only mode
if ($CheckOnly) {
    Write-Host "Step 0: Validating git state..." -ForegroundColor Yellow
    Test-GitWorkingDirectory
    Write-Host "[OK] Working directory is clean" -ForegroundColor Green
    Write-Host ""
    
    Write-Host "Step 1: Checking upstream remote..." -ForegroundColor Yellow
    $remotes = git remote -v
    if ($remotes -match "upstream") {
        Write-Host "[OK] Upstream remote already exists" -ForegroundColor Green
    } else {
        Write-Host "[WARN] Upstream remote not found. Add with: git remote add upstream $UPSTREAM_REPO" -ForegroundColor Yellow
    }
    Write-Host ""
    
    Write-Host "Step 2: Fetching latest release..." -ForegroundColor Yellow
    Invoke-GitCommand "git fetch upstream --tags --force" "Fetch upstream tags"
    
    try {
        $tags = @(git tag -l | Where-Object { $_ -match "^v\d+\.\d+\.\d+$" })
        if ($tags.Count -eq 0) {
            Write-Host "[ERROR] Could not find any stable release tags" -ForegroundColor Red
            exit 1
        }
        $latest_tag = $tags | Sort-Object { [Version]($_ -replace 'v', '') } | Select-Object -Last 1
    } catch {
        Write-Host "[ERROR] Failed to parse version tags: $_" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Latest upstream tag: $latest_tag" -ForegroundColor Green
    Write-Host ""
    
    Write-Host "Step 3: Analyzing branch differences..." -ForegroundColor Yellow
    $divergeCount = Show-BranchDifferences "main" "$CUSTOM_BRANCH" "Files in $CUSTOM_BRANCH not in main"
    
    Write-Host ""
    if ($divergeCount -gt 20) {
        Write-Host "[WARNING] High divergence detected ($divergeCount files)" -ForegroundColor Yellow
        Write-Host "Consider running with -Cleanup flag to remove unwanted core changes" -ForegroundColor Cyan
    } else {
        Write-Host "[OK] Divergence is manageable" -ForegroundColor Green
    }
    
    Write-Host ""
    Write-Host "=== Check Complete ===" -ForegroundColor Cyan
    Write-Host "No changes were made. Run without -CheckOnly to perform sync." -ForegroundColor Cyan
    exit 0
}

# Step 0: Validate Git State
Write-Host "Step 0: Validating git state..." -ForegroundColor Yellow
Test-GitWorkingDirectory
Write-Host "[OK] Working directory is clean" -ForegroundColor Green
Write-Host ""

# Step 1: Add the Original Repository as Upstream
Write-Host "Step 1: Checking upstream remote..." -ForegroundColor Yellow
$remotes = git remote -v
if ($remotes -match "upstream") {
    Write-Host "[OK] Upstream remote already exists" -ForegroundColor Green
} else {
    Write-Host "Adding upstream remote..."
    Invoke-GitCommand "git remote add upstream $UPSTREAM_REPO" "Add upstream remote"
    Write-Host "[OK] Upstream remote added" -ForegroundColor Green
}
Write-Host ""

# Step 2: Get Default Branch
Write-Host "Step 2: Detecting default branch..." -ForegroundColor Yellow
Invoke-GitCommand "git fetch origin" "Fetch origin"
$DEFAULT_BRANCH = Get-DefaultBranch
Write-Host "[OK] Default branch is: $DEFAULT_BRANCH" -ForegroundColor Green
Write-Host ""

# Step 3: Reset Main/Master Branch to Latest Release
Write-Host "Step 3: Fetching latest release from upstream..." -ForegroundColor Yellow
Invoke-GitCommand "git fetch upstream --tags --force" "Fetch upstream tags"

# Get the most recent stable release tag (excluding betas and pre-releases)
try {
    $tags = @(git tag -l | Where-Object { $_ -match "^v\d+\.\d+\.\d+$" })
    if ($tags.Count -eq 0) {
        Write-Host "[ERROR] Could not find any stable release tags matching v*.*.* pattern" -ForegroundColor Red
        exit 1
    }
    
    $latest_tag = $tags | Sort-Object { [Version]($_ -replace 'v', '') } | Select-Object -Last 1
} catch {
    Write-Host "[ERROR] Failed to parse version tags: $_" -ForegroundColor Red
    exit 1
}

Write-Host "Latest release tag: $latest_tag" -ForegroundColor Green
Write-Host ""

# Step 4: Checkout and Reset Default Branch
Write-Host "Step 4: Resetting $DEFAULT_BRANCH branch to $latest_tag..." -ForegroundColor Yellow
Invoke-GitCommand "git checkout $DEFAULT_BRANCH" "Checkout $DEFAULT_BRANCH"
Invoke-GitCommand "git reset --hard $latest_tag" "Reset to $latest_tag"
Write-Host "[OK] $DEFAULT_BRANCH branch reset successfully" -ForegroundColor Green
Write-Host ""

# Confirm before force push
Write-Host "Summary of changes:" -ForegroundColor Cyan
$diff_summary = git log --oneline -10 $DEFAULT_BRANCH
Write-Host $diff_summary
Write-Host ""

$confirm = Read-Host "Ready to force push $DEFAULT_BRANCH to origin. Continue? (y/N)"
if ($confirm -ne 'y' -and $confirm -ne 'Y') {
    Write-Host "[ABORTED] $DEFAULT_BRANCH branch updated locally but not pushed." -ForegroundColor Yellow
    exit 0
}

if ($TestMode) {
    Write-Host "[TEST MODE] Would force push $DEFAULT_BRANCH to origin" -ForegroundColor Cyan
} else {
    Write-Host ""
    Write-Host "Force pushing $DEFAULT_BRANCH..." -ForegroundColor Yellow
    Invoke-GitCommand "git push origin $DEFAULT_BRANCH --force" "Force push $DEFAULT_BRANCH"
}
Write-Host "[OK] $DEFAULT_BRANCH branch updated and pushed" -ForegroundColor Green
Write-Host ""

# Step 5: Merge Updates into Custom Branch
Write-Host "Step 5: Merging updates into $CUSTOM_BRANCH..." -ForegroundColor Yellow

# Verify custom branch exists
if (-not (git rev-parse --verify $CUSTOM_BRANCH 2>$null)) {
    Write-Host "[ERROR] Custom branch '$CUSTOM_BRANCH' does not exist" -ForegroundColor Red
    Write-Host "Available branches:" -ForegroundColor Yellow
    git branch -a
    exit 1
}

Invoke-GitCommand "git checkout $CUSTOM_BRANCH" "Checkout $CUSTOM_BRANCH"

Write-Host ""
Write-Host "Analyzing merge impact..." -ForegroundColor Yellow
Show-BranchDifferences "$CUSTOM_BRANCH" "$DEFAULT_BRANCH" "Changes that will be merged from $DEFAULT_BRANCH"

Write-Host ""
Write-Host "Attempting to merge $DEFAULT_BRANCH into $CUSTOM_BRANCH..." -ForegroundColor Yellow
git merge $DEFAULT_BRANCH --no-edit

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "[WARNING] Merge conflicts detected!" -ForegroundColor Yellow
    Write-Host ""
    
    $conflictFiles = @(git diff --name-only --diff-filter=U)
    Write-Host "Conflicted files ($($conflictFiles.Count)):" -ForegroundColor Yellow
    $conflictFiles | ForEach-Object { Write-Host "  - $_" }
    
    Write-Host ""
    Write-Host "Options:" -ForegroundColor Cyan
    Write-Host "  1. Resolve conflicts manually in your editor"
    Write-Host "  2. Abort merge and run with -Cleanup: git merge --abort && .\sync_fork.ps1 -Cleanup"
    Write-Host "  3. Keep your version: git checkout --ours <file> && git add <file>"
    Write-Host "  4. Keep main version: git checkout --theirs <file> && git add <file>"
    Write-Host ""
    Write-Host "After resolving, complete the merge with:" -ForegroundColor Cyan
    Write-Host "  git add . && git commit -m 'Merge: synced with upstream $latest_tag'" -ForegroundColor White
    Write-Host "  git push origin $CUSTOM_BRANCH" -ForegroundColor White
    Write-Host ""
    Write-Host "Or abort with: git merge --abort" -ForegroundColor Yellow
    exit 1
}

Write-Host "[OK] Merge completed successfully without conflicts" -ForegroundColor Green
Write-Host ""

# Confirm before pushing custom branch
$confirm_push = Read-Host "Push changes to origin/$CUSTOM_BRANCH? (y/N)"
if ($confirm_push -ne 'y' -and $confirm_push -ne 'Y') {
    Write-Host "[INFO] Changes not pushed locally. Run 'git push origin $CUSTOM_BRANCH' when ready." -ForegroundColor Yellow
    exit 0
}

if ($TestMode) {
    Write-Host "[TEST MODE] Would push $CUSTOM_BRANCH to origin" -ForegroundColor Cyan
} else {
    Write-Host ""
    Write-Host "Pushing $CUSTOM_BRANCH..." -ForegroundColor Yellow
    Invoke-GitCommand "git push origin $CUSTOM_BRANCH" "Push $CUSTOM_BRANCH"
}
Write-Host "[OK] Custom branch updated and pushed" -ForegroundColor Green
Write-Host ""

Write-Host "=== Sync Complete ===" -ForegroundColor Cyan
Write-Host "[OK] $DEFAULT_BRANCH branch synced to $latest_tag" -ForegroundColor Green
Write-Host "[OK] $CUSTOM_BRANCH merged with latest updates" -ForegroundColor Green
Write-Host ""
Write-Host "[INFO] Remember to test your customizations with the new code!" -ForegroundColor Yellow
