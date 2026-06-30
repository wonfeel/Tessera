// engine/chunk/ChunkStore.cpp
#include "engine/chunk/ChunkStore.h"
#include <algorithm>

Chunk* ChunkStore::get(ChunkCoord coord) {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_chunks.find(coord);
    return (it != m_chunks.end()) ? it->second.get() : nullptr;
}

ChunkStore::ChunkPtr ChunkStore::getShared(ChunkCoord coord) const {
    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_chunks.find(coord);
    return (it != m_chunks.end()) ? it->second : nullptr;
}

Chunk* ChunkStore::getOrCreateLocked(ChunkCoord coord, const Factory& factory) {
    auto it = m_chunks.find(coord);
    if (it != m_chunks.end()) return it->second.get();

    ChunkPtr chunk = factory(coord);
    Chunk* ptr = chunk.get();
    m_chunks[coord] = std::move(chunk);
    return ptr;
}

void ChunkStore::addActiveIfMissing(const ChunkCoord& coord) {
    auto it = std::find(m_activeChunks.begin(), m_activeChunks.end(), coord);
    if (it == m_activeChunks.end()) m_activeChunks.push_back(coord);
}

void ChunkStore::removeActive(const ChunkCoord& coord) {
    auto it = std::find(m_activeChunks.begin(), m_activeChunks.end(), coord);
    if (it != m_activeChunks.end()) m_activeChunks.erase(it);
}

void ChunkStore::eraseLocked(const ChunkCoord& coord) {
    removeActive(coord);
    m_chunks.erase(coord);
}
