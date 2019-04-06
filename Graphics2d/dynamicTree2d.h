#ifndef GRAPHICS2D_DYNAMIC_TREE_2D_H
#define GRAPHICS2D_DYNAMIC_TREE_2D_H

namespace lib2d
{
    const int nullnode = -1;

    struct node
    {
        bool isleaf() const
        {
            return child1 == nullnode;
        }

        b2AABB aabb;

        void* userData;

        union
        {
            int parent;
            int next;
        };

        int child1;
        int child2;

        int height;
    };


    class dynamicTree2d
    {
    public:
        dynamicTree2d() = default;
        ~dynamicTree2d() = default;
        //void* GetUserData(int proxyId) const;
        //const b2AABB& GetFatAABB(int proxyId) const;
        //template <typename T>
        //void Query(T* callback, const b2AABB& aabb) const;
        //template <typename T>
        //void RayCast(T* callback, const b2RayCastInput& input) const;
    };

    class b2DynamicTree
    {
        int CreateProxy(const b2AABB& aabb, void* userData);
        void DestroyProxy(int proxyId);
        bool MoveProxy(int proxyId, const b2AABB& aabb1, const b2Vec2& displacement);

        void Validate() const;
        int GetHeight() const;
        int GetMaxBalance() const;
        float32 GetAreaRatio() const;
        void RebuildBottomUp();
        void ShiftOrigin(const b2Vec2& newOrigin);
    private:
        int AllocateNode();
        void FreeNode(int node);
        void InsertLeaf(int node);
        void RemoveLeaf(int node);
        int Balance(int index);
        int ComputeHeight() const;
        int ComputeHeight(int nodeId) const;
        void ValidateStructure(int index) const;
        void ValidateMetrics(int index) const;
        int m_root;
        b2TreeNode* m_nodes;
        int m_nodeCount;
        int m_nodeCapacity;
        int m_freeList;
        uint m_path;
        int m_insertionCount;
    };
}

#endif //GRAPHICS2D_DYNAMIC_TREE_2D_H