#!/bin/bash

# Syncing JoeboyC2 WLED Fork with the Upstream Repository
# This script syncs the fork with the upstream WLED repository's latest release

set -e  # Exit on error

UPSTREAM_REPO="https://github.com/Aircoookie/WLED.git"
CUSTOM_BRANCH="JoeboyC2_Mods"

echo "=== Syncing Fork with Upstream WLED Repository ==="
echo ""

# Step 1: Add the Original Repository as Upstream
echo "Step 1: Checking upstream remote..."
if git remote -v | grep -q "upstream"; then
    echo "✓ Upstream remote already exists"
else
    echo "Adding upstream remote..."
    git remote add upstream "$UPSTREAM_REPO"
    echo "✓ Upstream remote added"
fi
echo ""

# Step 2: Reset Main Branch to Latest Release
echo "Step 2: Fetching latest release from upstream..."
git fetch upstream --tags

# Get the most recent stable release tag (excluding betas and pre-releases)
latest_tag=$(git tag -l | grep -E "^v[0-9]+\.[0-9]+\.[0-9]+$" | sort -V | tail -n 1)

if [ -z "$latest_tag" ]; then
    echo "❌ Error: Could not find latest release tag"
    exit 1
fi

echo "Latest release tag: $latest_tag"
echo ""

echo "Resetting main branch to $latest_tag..."
git checkout main
git reset --hard "$latest_tag"

echo ""
read -p "Ready to force push to origin/main. Continue? (y/N): " confirm
if [[ $confirm =~ ^[Yy]$ ]]; then
    git push origin main --force
    echo "✓ Main branch updated and pushed"
else
    echo "❌ Aborted. Main branch updated locally but not pushed."
    exit 1
fi
echo ""

# Step 3: Merge Updates into Custom Branch
echo "Step 3: Merging updates into $CUSTOM_BRANCH..."
git checkout "$CUSTOM_BRANCH"

echo ""
echo "Attempting to merge main into $CUSTOM_BRANCH..."
if git merge main --no-edit; then
    echo "✓ Merge completed successfully"
else
    echo ""
    echo "⚠️  Merge conflicts detected!"
    echo "Please resolve conflicts manually, then run:"
    echo "  git add ."
    echo "  git commit -m 'Merged updates from latest release'"
    echo "  git push origin $CUSTOM_BRANCH"
    exit 1
fi

echo ""
read -p "Push changes to origin/$CUSTOM_BRANCH? (y/N): " confirm_push
if [[ $confirm_push =~ ^[Yy]$ ]]; then
    git push origin "$CUSTOM_BRANCH"
    echo "✓ Custom branch updated and pushed"
else
    echo "❌ Changes not pushed. Run 'git push origin $CUSTOM_BRANCH' when ready."
    exit 1
fi

echo ""
echo "=== Sync Complete ==="
echo "✓ Main branch synced to $latest_tag"
echo "✓ $CUSTOM_BRANCH merged with latest updates"
echo ""
echo "⚠️  Remember to test your customizations with the new code!"