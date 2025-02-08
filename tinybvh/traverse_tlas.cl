// ============================================================================
//
//        T R A V E R S E _ T L A S
// 
// ============================================================================

// Here, the TLAS is expected to be in 'BVH_GPU' format, i.e. a 2-wide BVH
// in the layout proposed by Aila & Laine. This layout is chosen here for its 
// balance between efficient construction and traversal.
// For scenes with lots of BLAS nodes you may want to use a wider BVH, e.g.
// BVH4_GPU.

float4 traverse_tlas( global struct BVHNodeAlt* tlasNode, global unsigned* idx, const float4 O4, const float4 D4, const float4 rD4, const float tmax )
{
	// traverse BVH
	float4 hit;
	hit.x = tmax;
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		// fetch the node
		const float4 lmin = tlasNode[node].lmin, lmax = tlasNode[node].lmax;
		const float4 rmin = tlasNode[node].rmin, rmax = tlasNode[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const uint instIdx = idx[firstTri + i];
				const struct BLASInstance* inst = instances + instIdx;
				const float4 Oblas = (float4)( TransformPoint( O4.xyz, inst->invTransform ), 1 );
				const float4 Dblas = (float4)( TransformVector( D4.xyz, inst->invTransform ), 0 );
				const float4 rDblas = (float4)( 1 / Dblas.x, 1 / Dblas.y, 1 / Dblas.z, 1 );
				const global float4* blasNodes = instIdx == 0 ? bistroNodes : dragonNodes;
				const global float4* blasTris = instIdx == 0 ? bistroTris : dragonTris;
			#ifdef SIMD_AABBTEST
				const float4 h = traverse_cwbvh( blasNodes, blasTris, Oblas, Dblas, rDblas, hit.x );
			#else
				const float4 h = traverse_cwbvh( blasNodes, blasTris, Oblas.xyz, Dblas.xyz, rDblas.xyz, hit.x );
			#endif
				if (h.x < hit.x) 
				{
					hit = h;
					hit.w = as_float( as_uint( hit.w ) + (instIdx << 24) );
				}
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
		const float3 t1a = (lmin.xyz - O4.xyz) * rD4.xyz, t2a = (lmax.xyz - O4.xyz) * rD4.xyz;
		const float3 t1b = (rmin.xyz - O4.xyz) * rD4.xyz, t2b = (rmax.xyz - O4.xyz) * rD4.xyz;
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), hit.x );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), hit.x );
		float dist1 = tmina > tmaxa ? 1e30f : tmina;
		float dist2 = tminb > tmaxb ? 1e30f : tminb;
		// traverse nearest child first
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			unsigned t = left; left = right; right = t;
		}
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	// write back intersection result
	return hit;
}

bool isoccluded_tlas( global struct BVHNodeAlt* tlasNode, global unsigned* idx, const float4 O4, const float4 D4, const float4 rD4, const float tmax )
{
	// traverse BVH
	unsigned node = 0, stack[STACK_SIZE], stackPtr = 0;
	while (1)
	{
		// fetch the node
		const float4 lmin = tlasNode[node].lmin, lmax = tlasNode[node].lmax;
		const float4 rmin = tlasNode[node].rmin, rmax = tlasNode[node].rmax;
		const unsigned triCount = as_uint( rmin.w );
		if (triCount > 0)
		{
			// process leaf node
			const unsigned firstTri = as_uint( rmax.w );
			for (unsigned i = 0; i < triCount; i++)
			{
				const uint instIdx = idx[firstTri + i];
				const struct BLASInstance* inst = instances + instIdx;
				const float4 Oblas = (float4)( TransformPoint( O4.xyz, inst->invTransform ), 1 );
				const float4 Dblas = (float4)( TransformVector( D4.xyz, inst->invTransform ), 0 );
				const float4 rDblas = (float4)( 1 / Dblas.x, 1 / Dblas.y, 1 / Dblas.z, 1 );
				const global float4* blasNodes = instIdx == 0 ? bistroNodes : dragonNodes;
				const global float4* blasTris = instIdx == 0 ? bistroTris : dragonTris;
			#ifdef SIMD_AABBTEST
				if (isoccluded_cwbvh( blasNodes, blasTris, Oblas, Dblas, rDblas, D4.w )) return true;
			#else
				if (isoccluded_cwbvh( blasNodes, blasTris, Oblas.xyz, Dblas.xyz, rDblas.xyz, D4.w )) return true;
			#endif
			}
			if (stackPtr == 0) break; else node = stack[--stackPtr];
			continue;
		}
		unsigned left = as_uint( lmin.w ), right = as_uint( lmax.w );
		// child AABB intersection tests
		const float3 t1a = (lmin.xyz - O4.xyz) * rD4.xyz, t2a = (lmax.xyz - O4.xyz) * rD4.xyz;
		const float3 t1b = (rmin.xyz - O4.xyz) * rD4.xyz, t2b = (rmax.xyz - O4.xyz) * rD4.xyz;
		const float3 minta = fmin( t1a, t2a ), maxta = fmax( t1a, t2a );
		const float3 mintb = fmin( t1b, t2b ), maxtb = fmax( t1b, t2b );
		const float tmina = fmax( fmax( fmax( minta.x, minta.y ), minta.z ), 0 );
		const float tminb = fmax( fmax( fmax( mintb.x, mintb.y ), mintb.z ), 0 );
		const float tmaxa = fmin( fmin( fmin( maxta.x, maxta.y ), maxta.z ), tmax );
		const float tmaxb = fmin( fmin( fmin( maxtb.x, maxtb.y ), maxtb.z ), tmax );
		float dist1 = tmina > tmaxa ? 1e30f : tmina;
		float dist2 = tminb > tmaxb ? 1e30f : tminb;
		// traverse nearest child first
		if (dist1 > dist2)
		{
			float h = dist1; dist1 = dist2; dist2 = h;
			unsigned t = left; left = right; right = t;
		}
		if (dist1 == 1e30f) { if (stackPtr == 0) break; else node = stack[--stackPtr]; }
		else { node = left; if (dist2 != 1e30f) stack[stackPtr++] = right; }
	}
	// no hit found
	return false;
}