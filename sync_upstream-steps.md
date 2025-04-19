# Syncing JoeboyC2 WLED Fork with the Upstream Repository

This guide explains how to manually sync the fork with the upstream WLED repository's latest release, while also keeping the custom changes in the JoeboyC2_Mods branch. These steps have also been automated using a GitHub workflow that can be found in the fork.

---

## **Step 1: Add the Original Repository as Upstream**
Run the following commands to add the upstream WLED repository as a remote:

```bash
# Check the current setup remotes
git remote -v

# Add the upstream remote if not already added
git remote add upstream https://github.com/Aircoookie/WLED.git

# Fetch all tags from upstream
git fetch upstream --tags
```

---

## **Step 2: Sync with Latest Release**
1. Find the latest release tag:
   ```bash
   # Get the most recent stable release tag (excluding betas and pre-releases)
   latest_tag=$(git tag -l | grep -E "^v[0-9]+\.[0-9]+\.[0-9]+$" | sort -V | tail -n 1)
   echo "Latest tag: $latest_tag"
   ```

2. Switch to the `main` branch and merge the latest release:
   ```bash
   # Switch to main branch
   git checkout main
   
   # Merge the latest release tag
   git merge $latest_tag
   
   # Push the updated main branch
   git push origin main
   ```

---

## **Step 3: Merge Updates into Custom Branch**
1. Switch to custom branch (`JoeboyC2_Mods`):
   ```bash
   git checkout JoeboyC2_Mods
   ```

2. Merge the updated `main` branch into custom branch:
   ```bash
   git merge main
   ```

3. Resolve any conflicts if prompted, then commit the changes:
   ```bash
   git add .
   git commit -m "Merged updates from latest release"
   ```

4. Push the changes to your custom branch:
   ```bash
   git push origin JoeboyC2_Mods
   ```

5. Test customizations with new code to ensure everything works as expected.

---

## **Optional: Use Rebasing Instead of Merging**
If you prefer a cleaner commit history, you can rebase your branch instead of merging:

```bash
# Rebase your custom branch on top of the updated main branch
git rebase main
```

Resolve conflicts as they arise during the rebase process.

---

## **Automated Workflow**

This repository includes a GitHub Actions workflow to automate the process of syncing the JoeboyC2 fork with the latest release. The workflow is triggered manually and performs the following steps automatically:
1. Fetches the latest release tag from the upstream repository.
2. Merges the latest release into your `main` branch.
3. Merges the `main` branch into your custom branch (`JoeboyC2_Mods`).

To trigger the workflow:
1. Go to the **Actions** tab in this repository.
2. Select the **Sync Upstream** workflow.
3. Click on **Run workflow** and follow the prompts.

You can find the workflow definition in `.github/workflows/sync_upstream.yml`.

---