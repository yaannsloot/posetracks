/*
Copyright (C) 2024 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

namespace me {
	
	namespace data {
		
		template <typename T>
		class Edge {
		public:
			Edge(const T& a, const T& b) { this->a = a; this->b = b; };
			T getA() const { return a; }
			T getB() const { return b; }
		private:
			T a;
			T b;
		};

		template <typename T>
		bool operator==(const Edge<T>& a, const Edge<T>& b) {
			return (a.getA() == b.getA() && a.getB() == b.getB()) || (a.getA() == b.getB() && a.getB() == b.getA());
		}

		template <typename T>
		bool operator!=(const Edge<T>& a, const Edge<T>& b) {
			return !(a == b);
		}

		template <typename T>
		bool operator< (const Edge<T>& a, const Edge<T>& b) {
			return (a.getA() < b.getA() || a.getB() < b.getB() || a.getA() < b.getB() || a.getB() < b.getA()) && a != b;
		}

		template <typename T>
		class Graph {
		public:
			std::set<T> getVerticies();
			bool checkContinuity();
			std::stack<T> getVertexPathDFS(const T& a, const T& b);
			std::set<Edge<T>> getEdges();
			std::set<T> getAdjacentVerticies(const T& v);
			void addEdge(const T& a, const T& b);
			void addEdge(const Edge<T>& e);
			void removeEdge(const T& a, const T& b);
			void removeEdge(const Edge<T>& e);
			bool empty();
		private:
			std::set<Edge<T>> edges;
			void recursive_traverse(std::set<T>& visited, const T& current);
			void dfs(std::set<T>& visited, const T& current, const T& target, std::stack<T>& path, bool& found);
		};

		template <typename T>
		std::set<T> Graph<T>::getVerticies() {
			std::set<T> result;
			for (auto it = edges.begin(); it != edges.end(); it++) {
				result.emplace(it->getA());
				result.emplace(it->getB());
			}
			return result;
		}

		template <typename T>
		bool Graph<T>::checkContinuity() {
			if (!edges.empty()) {
				std::set<T> verts = getVerticies();
				std::set<T> reachable;
				T first = *verts.begin();
				recursive_traverse(reachable, first);
				if (reachable.size() == verts.size())
					return true;
			}
			return false;
		}

		template <typename T>
		std::stack<T> Graph<T>::getVertexPathDFS(const T& a, const T& b) {
			std::stack<T> result;
			if (!edges.empty()) {
				std::set<T> visited;
				bool found = false;
				dfs(visited, a, b, result, found);
			}
			return result;
		}

		template <typename T>
		std::set<Edge<T>> Graph<T>::getEdges() {
			return edges;
		}

		template <typename T>
		std::set<T> Graph<T>::getAdjacentVerticies(const T& v) {
			std::set<T> result;
			for (auto it = edges.begin(); it != edges.end(); it++) {
				if ((v == it->getA() || v == it->getB()) && it->getA() != it->getB()) {
					if (v == it->getA())
						result.emplace(it->getB());
					else
						result.emplace(it->getA());
				}
			}
			return result;
		}

		template <typename T>
		void Graph<T>::addEdge(const T& a, const T& b) {
			edges.emplace(Edge<T>(a, b));
		}

		template <typename T>
		void Graph<T>::addEdge(const Edge<T>& e) {
			edges.emplace(e);
		}

		template <typename T>
		void Graph<T>::removeEdge(const T& a, const T& b) {
			edges.erase(std::remove(edges.begin(), edges.end(), Edge<T>(a, b)));
		}

		template <typename T>
		void Graph<T>::removeEdge(const Edge<T>& e) {
			edges.erase(std::remove(edges.begin(), edges.end(), e));
		}

		template <typename T>
		bool Graph<T>::empty() {
			return edges.empty();
		}

		template <typename T>
		void Graph<T>::recursive_traverse(std::set<T>& visited, const T& current) {
			visited.emplace(current);
			std::set<T> adjacent = getAdjacentVerticies(current);
			for (auto it = adjacent.begin(); it != adjacent.end(); it++) {
				if (std::find(visited.begin(), visited.end(), *it) == visited.end())
					recursive_traverse(visited, *it);
			}
		}

		template <typename T>
		void Graph<T>::dfs(std::set<T>& visited, const T& current, const T& target, std::stack<T>& path, bool& found) {
			visited.emplace(current);
			if (current == target) {
				found = true;
			}
			if (!found) {
				std::set<T> adjacent = getAdjacentVerticies(current);
				for (auto it = adjacent.begin(); it != adjacent.end(); it++) {
					if (std::find(visited.begin(), visited.end(), *it) == visited.end())
						dfs(visited, *it, target, path, found);
					if (found)
						break;
				}
			}
			if (found)
				path.push(current);
		}
		
	}
	
}