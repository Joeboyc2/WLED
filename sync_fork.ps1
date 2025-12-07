# Syncing JoeboyC2 WLED Fork with the Upstream Repository
# This script syncs the fork with the upstream WLED repository's latest release

$ErrorActionPreference = "Stop"

$UPSTREAM_REPO = "https://github.com/Aircoookie/WLED.git"
$CUSTOM_BRANCH = "JoeboyC2_Mods"

Write-Host "=== Syncing Fork with Upstream WLED Repository ===" -ForegroundColor Cyan
Write-Host ""

# Step 1: Add the Original Repository as Upstream
Write-Host "Step 1: Checking upstream remote..." -ForegroundColor Yellow
$remotes = git remote -v
if ($remotes -match "upstream") {
    Write-Host "[OK] Upstream remote already exists" -ForegroundColor Green
} else {
    Write-Host "Adding upstream remote..."
    git remote add upstream $UPSTREAM_REPO
    Write-Host "[OK] Upstream remote added" -ForegroundColor Green
}
Write-Host ""

# Step 2: Reset Main Branch to Latest Release
Write-Host "Step 2: Fetching latest release from upstream..." -ForegroundColor Yellow
git fetch upstream --tags

# Get the most recent stable release tag (excluding betas and pre-releases)
$tags = git tag -l | Where-Object { $_ -match "^v\d+\.\d+\.\d+$" }
$latest_tag = $tags | Sort-Object { [Version]($_ -replace 'v', '') } | Select-Object -Last 1

if (-not $latest_tag) {
    Write-Host "[ERROR] Could not find latest release tag" -ForegroundColor Red
    exit 1
}

Write-Host "Latest release tag: $latest_tag" -ForegroundColor Green
Write-Host ""

Write-Host "Resetting main branch to $latest_tag..."
git checkout main
git reset --hard $latest_tag

Write-Host ""
$confirm = Read-Host "Ready to force push to origin/main. Continue? (y/N)"
if ($confirm -eq 'y' -or $confirm -eq 'Y') {
    git push origin main --force
    Write-Host "[OK] Main branch updated and pushed" -ForegroundColor Green
} else {
    Write-Host "[ABORTED] Main branch updated locally but not pushed." -ForegroundColor Red
    exit 1
}
Write-Host ""

# Step 3: Merge Updates into Custom Branch
Write-Host "Step 3: Merging updates into $CUSTOM_BRANCH..." -ForegroundColor Yellow
git checkout $CUSTOM_BRANCH

Write-Host ""
Write-Host "Attempting to merge main into $CUSTOM_BRANCH..."
try {
    git merge main --no-edit
    Write-Host "[OK] Merge completed successfully" -ForegroundColor Green
} catch {
    Write-Host ""
    Write-Host "[WARNING] Merge conflicts detected!" -ForegroundColor Yellow
    Write-Host "Please resolve conflicts manually, then run:"
    Write-Host "  git add ."
    Write-Host "  git commit -m 'Merged updates from latest release'"
    Write-Host "  git push origin $CUSTOM_BRANCH"
    exit 1
}

Write-Host ""
$confirm_push = Read-Host "Push changes to origin/$CUSTOM_BRANCH? (y/N)"
if ($confirm_push -eq 'y' -or $confirm_push -eq 'Y') {
    git push origin $CUSTOM_BRANCH
    Write-Host "[OK] Custom branch updated and pushed" -ForegroundColor Green
} else {
    Write-Host "[INFO] Changes not pushed. Run 'git push origin $CUSTOM_BRANCH' when ready." -ForegroundColor Yellow
    exit 1
}

Write-Host ""
Write-Host "=== Sync Complete ===" -ForegroundColor Cyan
Write-Host "[OK] Main branch synced to $latest_tag" -ForegroundColor Green
Write-Host "[OK] $CUSTOM_BRANCH merged with latest updates" -ForegroundColor Green
Write-Host ""
Write-Host "[INFO] Remember to test your customizations with the new code!" -ForegroundColor Yellow
