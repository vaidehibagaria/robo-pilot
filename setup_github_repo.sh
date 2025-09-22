#!/bin/bash

# RCM Robot Control Framework - GitHub Repository Setup Script

echo "🚀 Setting up GitHub repository for RCM Robot Control Framework"
echo "=============================================================="

# Check if git is initialized
if [ ! -d ".git" ]; then
    echo "📁 Initializing Git repository..."
    git init
    echo "✓ Git repository initialized"
else
    echo "✓ Git repository already exists"
fi

# Add all files
echo "📝 Adding files to Git..."
git add .

# Create initial commit
echo "💾 Creating initial commit..."
git commit -m "Initial commit: RCM Robot Control Framework

- Add core framework with RCM server, tool generator, and agent bridge
- Add GPT integration for natural language robot control
- Add Docker support with automated setup
- Add comprehensive documentation and examples
- Add CI/CD pipeline with GitHub Actions
- Add support for multiple robot types (TurtleBot, UR5, Panda)
- Add real-time position tracking and path visualization"

echo "✓ Initial commit created"

# Check if remote origin exists
if git remote get-url origin > /dev/null 2>&1; then
    echo "✓ Remote origin already configured"
    echo "   URL: $(git remote get-url origin)"
else
    echo "⚠️  No remote origin configured"
    echo "   Please add your GitHub repository URL:"
    echo "   git remote add origin https://github.com/yourusername/rcm-robot-control.git"
fi

echo ""
echo "🎉 GitHub repository setup complete!"
echo ""
echo "Next steps:"
echo "1. Create a new repository on GitHub: https://github.com/new"
echo "2. Add the remote origin:"
echo "   git remote add origin https://github.com/yourusername/rcm-robot-control.git"
echo "3. Push to GitHub:"
echo "   git push -u origin main"
echo "4. Enable GitHub Actions in your repository settings"
echo "5. Add your OpenAI API key as a repository secret (optional)"
echo ""
echo "📚 Repository includes:"
echo "   ✓ Complete RCM framework"
echo "   ✓ Docker containerization"
echo "   ✓ GitHub Actions CI/CD"
echo "   ✓ Comprehensive documentation"
echo "   ✓ Multiple robot examples"
echo "   ✓ MIT License"
echo ""
echo "🔗 Share your repository:"
echo "   https://github.com/yourusername/rcm-robot-control"

