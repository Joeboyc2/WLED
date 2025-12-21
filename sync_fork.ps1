# Syncing JoeboyC2 WLED Fork with the Upstream WLED Repository
# 
# Features:
# - Validates git state before operations
# - Proper merge conflict detection
# - Safety checks before force push
# - Automatic default branch detection
# - Comprehensive error handling
# 
# Usage:
#   .\sync_fork.ps1                    # Run normally
#   .\sync_fork.ps1 -TestMode          # Test without making changes
#   .\sync_fork.ps1 -Verbose           # Show debug output

param(
    [switch]$TestMode = $false,
    [switch]$Verbose = $false
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

Write-Host "=== Syncing Fork with Upstream WLED Repository ===" -ForegroundColor Cyan
if ($TestMode) {
    Write-Host "[TEST MODE] - No changes will be made" -ForegroundColor Cyan
}
Write-Host ""

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
Invoke-GitCommand "git fetch upstream --tags" "Fetch upstream tags"

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
Write-Host "Attempting to merge $DEFAULT_BRANCH into $CUSTOM_BRANCH..." -ForegroundColor Yellow
git merge $DEFAULT_BRANCH --no-edit

if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "[WARNING] Merge conflicts detected!" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Conflicted files:" -ForegroundColor Yellow
    git diff --name-only --diff-filter=U
    Write-Host ""
    Write-Host "Next steps:" -ForegroundColor Cyan
    Write-Host "1. Resolve conflicts in your editor"
    Write-Host "2. Stage resolved files: git add <file>"
    Write-Host "3. Complete the merge: git commit -m 'Merge: synced with upstream $latest_tag'"
    Write-Host "4. Push the branch: git push origin $CUSTOM_BRANCH"
    Write-Host ""
    Write-Host "Or abort the merge with: git merge --abort"
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
