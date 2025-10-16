#!/bin/bash
# Mega-Robot GitHub Upload Preparation Script
# Cleans temporary files and verifies repository structure before uploading

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_DIR"

echo "=========================================="
echo "Mega-Robot GitHub Upload Preparation"
echo "=========================================="
echo ""

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: Clean temporary files
echo "Step 1: Cleaning temporary files..."
echo "-----------------------------------"

CLEANED=0

# Remove TF frames PDFs and graphs
if ls frames*.pdf frames*.gv 1> /dev/null 2>&1; then
    echo -e "${YELLOW}Removing TF frames files...${NC}"
    rm -f frames*.pdf frames*.gv
    ((CLEANED++))
fi

# Clean build artifacts (if not in .gitignore)
if [ -d "build" ] && [ ! -f ".gitignore" ]; then
    echo -e "${YELLOW}Warning: .gitignore not found, build/ should be ignored${NC}"
fi

# Remove Python cache
if find . -type d -name "__pycache__" | grep -q .; then
    echo -e "${YELLOW}Removing Python cache...${NC}"
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    ((CLEANED++))
fi

# Remove temporary logs
if [ -f "/tmp/bringup_test.log" ] || [ -f "/tmp/mapping_test.log" ]; then
    echo -e "${YELLOW}Note: Temporary logs exist in /tmp (not tracked)${NC}"
fi

if [ $CLEANED -eq 0 ]; then
    echo -e "${GREEN}✓ No temporary files to clean${NC}"
else
    echo -e "${GREEN}✓ Cleaned $CLEANED temporary items${NC}"
fi
echo ""

# Step 2: Verify required files
echo "Step 2: Verifying required files..."
echo "-----------------------------------"

MISSING=0

check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✓${NC} $1"
    else
        echo -e "${RED}✗${NC} $1 (MISSING)"
        ((MISSING++))
    fi
}

check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✓${NC} $1/"
    else
        echo -e "${RED}✗${NC} $1/ (MISSING)"
        ((MISSING++))
    fi
}

# Essential files
check_file "README.md"
check_file "LICENSE"
check_file ".gitignore"

# Documentation
check_dir "docs"
check_file "docs/README_TEST.md"
check_file "docs/project_plan.md"
check_file "docs/AGENTS.md"
check_file "docs/DIRECTORY_STRUCTURE.md"
check_file "docs/GITHUB_UPLOAD_GUIDE.md"

# Scripts
check_dir "scripts"
check_file "scripts/pre_test_check.sh"
check_file "scripts/test_bringup.sh"
check_file "scripts/test_mapping.sh"
check_file "scripts/test_navigation.sh"

# Source packages
check_dir "src"
check_dir "src/t_robot_bringup"
check_dir "src/t_robot_slam"

echo ""
if [ $MISSING -gt 0 ]; then
    echo -e "${RED}✗ Missing $MISSING required files/directories${NC}"
    exit 1
else
    echo -e "${GREEN}✓ All required files present${NC}"
fi
echo ""

# Step 3: Check .gitignore coverage
echo "Step 3: Checking .gitignore coverage..."
echo "---------------------------------------"

IGNORED_OK=0

check_ignored() {
    local dir=$1
    if [ -d "$dir" ]; then
        if grep -q "^$dir/\$" .gitignore 2>/dev/null; then
            echo -e "${GREEN}✓${NC} $dir/ is ignored"
            ((IGNORED_OK++))
        else
            echo -e "${YELLOW}⚠${NC} $dir/ exists but not in .gitignore (should be added)"
        fi
    fi
}

check_ignored "build"
check_ignored "install"
check_ignored "log"

if [ $IGNORED_OK -eq 3 ]; then
    echo -e "${GREEN}✓ Build artifacts properly configured in .gitignore${NC}"
fi
echo ""

# Step 4: Estimate repository size
echo "Step 4: Estimating repository size..."
echo "-------------------------------------"

# Calculate size excluding ignored directories
REPO_SIZE=$(du -sh --exclude=build --exclude=install --exclude=log --exclude=.git . 2>/dev/null | cut -f1)
echo "Estimated upload size: ${GREEN}$REPO_SIZE${NC}"

# Warn if too large
REPO_SIZE_MB=$(du -sm --exclude=build --exclude=install --exclude=log --exclude=.git . 2>/dev/null | cut -f1)
if [ "$REPO_SIZE_MB" -gt 100 ]; then
    echo -e "${YELLOW}⚠ Repository is quite large (${REPO_SIZE_MB}MB)${NC}"
    echo "  Consider using Git LFS for large files (rosbag, databases)"
fi
echo ""

# Step 5: Check for sensitive information
echo "Step 5: Checking for sensitive information..."
echo "---------------------------------------------"

SENSITIVE_FOUND=0

# Check for common sensitive patterns
if grep -r "password\|secret\|token" src/ --include="*.yaml" --include="*.py" --include="*.cpp" 2>/dev/null | grep -v "# password" | grep -q .; then
    echo -e "${RED}⚠ Potential sensitive information found in source files${NC}"
    ((SENSITIVE_FOUND++))
fi

# IP addresses are OK for internal network (192.168.x.x)
if grep -r "192.168" src/ --include="*.yaml" 2>/dev/null | grep -q .; then
    echo -e "${GREEN}✓${NC} Internal IP addresses found (192.168.x.x - OK for public repo)"
fi

if [ $SENSITIVE_FOUND -eq 0 ]; then
    echo -e "${GREEN}✓ No obvious sensitive information detected${NC}"
else
    echo -e "${RED}✗ Please review and remove sensitive information before uploading${NC}"
    exit 1
fi
echo ""

# Step 6: Generate upload summary
echo "Step 6: Upload summary..."
echo "------------------------"

FILE_COUNT=$(find src/ docs/ scripts/ -type f | wc -l)
DIR_COUNT=$(find src/ docs/ scripts/ -type d | wc -l)

echo "Files to upload: ${GREEN}$FILE_COUNT${NC}"
echo "Directories: ${GREEN}$DIR_COUNT${NC}"
echo ""

# List main packages
echo "ROS 2 Packages:"
for pkg in src/*/package.xml; do
    PKG_NAME=$(basename $(dirname $pkg))
    echo "  - $PKG_NAME"
done
echo ""

# Step 7: Git status check
echo "Step 7: Git status check..."
echo "---------------------------"

if [ -d ".git" ]; then
    echo -e "${GREEN}✓${NC} Git repository already initialized"

    # Check for uncommitted changes
    if ! git diff-index --quiet HEAD -- 2>/dev/null; then
        echo -e "${YELLOW}⚠ Uncommitted changes detected${NC}"
        echo "Run 'git status' to see changes"
    else
        echo -e "${GREEN}✓${NC} No uncommitted changes"
    fi

    # Check remote
    if git remote -v | grep -q "origin"; then
        REMOTE_URL=$(git remote get-url origin)
        echo -e "${GREEN}✓${NC} Remote configured: $REMOTE_URL"
    else
        echo -e "${YELLOW}⚠${NC} No remote configured"
        echo "Run: git remote add origin https://github.com/yeyanle6/Mega-Robot.git"
    fi
else
    echo -e "${YELLOW}⚠${NC} Git repository not initialized"
    echo "Run: git init"
fi
echo ""

# Final summary
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""

if [ $MISSING -eq 0 ] && [ $SENSITIVE_FOUND -eq 0 ]; then
    echo -e "${GREEN}✓ Repository is ready for GitHub upload!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Review docs/GITHUB_UPLOAD_GUIDE.md for detailed instructions"
    echo "  2. Initialize Git (if not done): git init"
    echo "  3. Add files: git add ."
    echo "  4. Commit: git commit -m 'feat: Initial commit - Mega-Robot v0.9'"
    echo "  5. Add remote: git remote add origin https://github.com/yeyanle6/Mega-Robot.git"
    echo "  6. Push: git push -u origin main"
    echo ""
else
    echo -e "${RED}✗ Issues found, please fix before uploading${NC}"
    exit 1
fi

exit 0
